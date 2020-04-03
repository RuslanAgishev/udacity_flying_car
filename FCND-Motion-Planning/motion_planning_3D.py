#!/usr/bin/env python

import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, a_star_graph, heuristic, create_grid, Graph, prune_path, find_graph_nearest_node
from sampling import Sampler
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, graph):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.graph = graph

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            local_position_NED = self.local_position
            local_position_NED[2] *= (-1.0)
            if np.linalg.norm(self.target_position[0:3] - local_position_NED[0:3]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        SAFETY_DISTANCE = 5

        # read lat0, lon0 from colliders into floating point values
        #lat0, lon0, alt0 = 37.792480, -122.397450, 0.
        with open('colliders.csv') as f:
            origin_pos_data = f.readline().split(',')
        lat0 = float(origin_pos_data[0].strip().split(' ')[1])
        lon0 = float(origin_pos_data[1].strip().split(' ')[1])
        alt0 = 0.0

        # set home position to (lon0, lat0, 0)
        self.set_home_position(longitude=lon0, latitude=lat0, altitude=alt0)

        # convert to current local position using global_to_local()
        start_local = global_to_local(self.global_position, self.global_home)
        graph_start = find_graph_nearest_node(self.graph, start_local)
        
        # goal_local = (start_local[0]+250., start_local[1]+15., 10.)
        # graph_goal = find_graph_nearest_node(self.graph, goal_local)
        
        # Choosing goal position randomly
        random_index = np.random.randint(len(self.graph.nodes))
        graph_goal = list(self.graph.nodes)[random_index]

        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))        
        
        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', graph_start, graph_goal)
        path, _ = a_star_graph(self.graph, heuristic, tuple(graph_start), tuple(graph_goal))
    
        # Convert path to waypoints
        waypoints = [[int(p[0]), int(p[1]), int(p[2]), 0] for p in path]
        
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()




def create_prm_graph(data):
    # Define a grid for a particular altitude and safety margin around obstacles
    print('Creating a grid...')
    grid, _, _ = create_grid(data, 5, 5)
    # Build PRM graph
    print('Sampling collision free points...')
    sampler = Sampler(data)
    polygons = sampler._polygons
    # sampling 300 points and removing ones conflicting with obstacles.
    nodes = sampler.sample(300)
    print('Building a PRM graph...')
    graph = Graph(nodes, polygons)
    graph.create_graph(10)
    print('Done!')
    return graph



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)

    # Read in obstacle map
    print('Reading obstacles map and constructing PRM graph...')
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    graph = create_prm_graph(data)

    drone = MotionPlanning(conn, graph)
    time.sleep(1)
    drone.start()



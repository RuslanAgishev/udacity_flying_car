import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.calculate_box()
        self.home_pose = np.array(self.global_position) # remember home position
        self.wp_tol = 0.2 # [m], distance when wp is considered as passed

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()

    def velocity_callback(self):
        pass

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                # now just passively waiting for the pilot to change these attributes
                # once the pilot changes, need to update our internal state
                if self.guided:
                    self.flight_state = States.ARMING
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.LANDING:
                # check if the pilot has changed the armed and control modes
                # if so (and the script no longer in control) stop the mission
                if not self.armed and not self.guided:
                    self.stop()
                    self.in_mission = False
            elif self.flight_state == States.DISARMING:
                # no longer want the vehicle to handle the disarming and releasing control
                # that will be done by the pilot
                pass

    def calculate_box(self):
        print("Setting Home")
        cp = np.array([self.local_position[0], self.local_position[1], -self.local_position[2]])  # get the current local position -> note we need to change the sign of the down coordinate to be altitude
        local_waypoints = [cp + [10.0, 0.0, 3.0], cp + [10.0, 10.0, 3.0], cp + [0.0, 10.0, 3.0], cp + [0.0, 0.0, 3.0]]
        self.all_waypoints local_waypoints
        return local_waypoints

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.home_pose = np.array(self.global_position)
        self.set_home_position(self.home_pose[0],
                               self.home_pose[1],
                               self.home_pose[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.next_wp = np.array(self.all_waypoints.pop())
        # self.next_wp[:3] += self.home_pose
        self.cmd_position(*self.next_wp)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()
    # Connection to Intel Aero drone example
    conn = MavlinkConnection('udp:192.168.1.2:14550', PX4=True, threaded=False) # IP of the ground station (laptop)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()

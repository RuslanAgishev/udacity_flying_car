## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

# Required Steps:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Send waypoints to the flight controller.
8. Enjoy the flight.

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Main functionality is provided in `motion_planning_2D.py` (or `motion_planning_3D.py`) and `planning_utils.py`
In motion planning scripts the UAV transition states logic is implemented as well as sending waypoints commands to the flight controller.
Planning utils is dedicated to provide main path planning functions, which are responsible for building a path ob the given map. Two approaches are considered in this work (planar: A* planning on a grid, spatial: A* planning on a PRM-graph).


### Implementing Planar Path Planning Algorithm (A* star 2D on a grid).

Main functionality is implemented [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_2D.py).

In order to test the planning solution:
    1. start the [simulator](https://github.com/udacity/FCND-Simulator-Releases)
    2. run the planning script: ```python motion_planning_2D.py```

#### 1. Set your global home position
Read provided `colliders.csv` file and extract from it initial global position as latitude / longitude coordinates.
```python
#lat0, lon0, alt0 = 37.792480, -122.397450, 0.
with open('colliders.csv') as f:
    origin_pos_data = f.readline().split(',')
lat0 = float(origin_pos_data[0].strip().split(' ')[1])
lon0 = float(origin_pos_data[1].strip().split(' ')[1])
alt0 = 0.0
self.set_home_position(longitude=lon0, latitude=lat0, altitude=alt0)
```

#### 2. Set your current local position
Determining drone local position relative to global home let us planning the route on the map starting from the current location.

```python
# convert to current local position using global_to_local()
start_ne = global_to_local(self.global_position, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as determined on previous step local position (in meters relative to home) is found, it is further transformed to position on the grid map.

Firstly we read map data (obstacles locations and their size).
```python
# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
```

Then we create a planar grid representation of occupied and free space. This is done by taking a slice from 2.5-map at a particular constant flight height (defined by variable ```TARGET_ALTITUDE```). Obstacles on the map are inflated by the ```SAFETY_DISTANCE``` constant value in X- and Y- directions. 
```python
# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
```

The next step is to transform local drone position (in meters) to location on a grid in pixels (or number of 2D-cells relative to down-left grid corner).
```python
# Define starting point on the grid (this is just grid center)
grid_start = ( int(start_ne[0])-north_offset, int(start_ne[1])-east_offset )
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid. Ones random obstacles free goal location is determined, we can run our planner to construct a path from the drone's location. Random choice of the goal position let us test the planner without changing the goal location every time manually.

First of all here, we define an angular range for latitudial and longitudial motions relative to home position (in degrees).
```python
angular_range = 0.005 # lat/lon range to localize a goal relative to home global pose
```
Then we define global goal location randomly (altitude is fixed for planar motion).
```python
lat_goal = lat0 + (2*np.random.rand()-1)*angular_range
lon_goal = lon0 + (2*np.random.rand()-1)*angular_range
alt_goal = -TARGET_ALTITUDE
global_goal_pose = np.array([lon_goal, lat_goal, alt_goal])
```
We transform further the global goal position into location on the given grid.
```python
goal_ne = global_to_local(global_goal_pose, global_home_pose)
goal_x, goal_y = int(goal_ne[0])-north_offset, int(goal_ne[1])-east_offset
```
Ones it is done we check if the suggested location is valid (not lies inside an obstacle). We do this step until a collision-free location is chosen.
```python
while grid[goal_x, goal_y] == 1:
    lat_goal = lat0 + (2*np.random.rand()-1)*angular_range
    lon_goal = lon0 + (2*np.random.rand()-1)*angular_range
    alt_goal = -TARGET_ALTITUDE
    global_goal_pose = np.array([lon_goal, lat_goal, alt_goal])
    goal_ne = global_to_local(global_goal_pose, global_home_pose)
    goal_x, goal_y = int(goal_ne[0])-north_offset, int(goal_ne[1])-east_offset
grid_goal = (goal_x, goal_y)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome.

Define actions and weights for them, [src](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/planning_utils.py#L58):
```python
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```
Check whether the actions are valid, [src](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/planning_utils.py#L83):
```python
if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```

#### 6. Cull waypoints 
For this step we can use a collinearity test or ray tracing method like [Bresenham](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm). The idea is simply to prune your path of unnecessary waypoints.

Here we check if it is possible to delete one of the three neighbour points if they meet collinearity criterea.
```python
def prune_path(path, eps=1e-6):
    pruned_path = [p for p in path]  
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3, eps):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
```

The collinearity condition is implemented in the following method. Here we check if the matrix constructed by 3 points have full rank. In case if it is not satisfied (equivalent to matrix determinant is equal to zero) the 3 points must lie on one straight line. We also add a threshold (```eps=0.5```) to handle the case when the points are approximaly collinear.
```python
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=0.5):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon
```
We check each 3 neighbouring points in our path to prune it and ommit unnecessary waypoints. This will let us avoid quadrotor slow and jerky movements when the waypoints are very dense relative to each other.

Ones all the necessary preflight steps are implemented, it is important to visualize the constructed path prior to the flight.

<img src="https://github.com/RuslanAgishev/udacity_flying_car/blob/master/figures/a_star_city.png" width="800"/>

Here you can see the map slice at altitude 5 m (as occupancy grid) and a path from start to goal construct with the A* algorithm.

Another possibility, is to represent the environment with graphs. For example, using [Voronoi diagrams](https://en.wikipedia.org/wiki/Voronoi_diagram) to build collision free roads, which intersections could serve as waupoints.
Ones such a graph is constructed, we could run A* on the graph in order to find a path from the vertex closest to drone's current location to the goal vertex. Implementation details could be found [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/Path_Planning/2D/Voronoi.ipynb).

<img src="https://github.com/RuslanAgishev/udacity_flying_car/blob/master/figures/voronoy_graph_a_star_search.png" width="800"/>

# Extra Challenges: Planning waypoint trajectories in 3D (PRM).

<img src="https://github.com/RuslanAgishev/udacity_flying_car/blob/master/figures/prm_unity.png" width="800"/>

One possible options, that helps to move into 3D and construct our trajectories in space rather than only at a constant height, is by constructing a PRM graph of sampled collision free points in our environment.

Main functionality is implemented [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py).

In order to test the planning solution:
    1. start the [simulator](https://github.com/udacity/FCND-Simulator-Releases)
    2. run the planning script: ```python motion_planning_3D.py```

3D implementation is very similar to a planar case. I will highlight here only the main differences.

1. Based on the obstacles grid map, we sample collision free points (possible PRM graph nodes) [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py#L184). We also create obstacles [polygons](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py#L185) in order to test if it is possible to connect neighbouring sampled nodes with straight lines.

2. Create  PRM gragh as our feasible paths representation, [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py#L189). Our MotionPlanning class takes the graph as [another input](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py#L209). The graph is constructed prior to flight, as it could take quite a long time.

3. It is also important to modify A* planning algorithm to work with graph representation. This is done [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/17decbe8e48e3e28756c89cc44f828921e1842f3/FCND-Motion-Planning/planning_utils.py#L229). Ones the path is built, we exctract waypoints out of it and send them to flight controller [as usual](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py#L156).

4. Obtained space-waypoints should be followed in all 3 directions (angles are not included in current implementation). That is why a waypoint reaching criterea should be [modified](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/motion_planning_3D.py#L54).

Utility functions for PRM planning are implemented [here](https://github.com/RuslanAgishev/udacity_flying_car/blob/master/FCND-Motion-Planning/planning_utils.py#L198).

### Execute the flight
  
Flight results in the unity simulator could be found in [my google drive](https://drive.google.com/open?id=1XEN0o8oCgfHH_emuToS5IdMiRcSzkxxC).



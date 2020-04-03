## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...


### Implementing Your Path Planning Algorithm

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
Here as long as you successfully determine your local position relative to global home you'll be all set.

```python
# convert to current local position using global_to_local()
start_ne = global_to_local(self.global_position, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

```python
# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

# Define starting point on the grid (this is just grid center)
grid_start = ( int(start_ne[0])-north_offset, int(start_ne[1])-east_offset )
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

```python
angular_range = 0.005 # lat/lon range to localize a goal relative to home global pose
lat_goal = lat0 + (2*np.random.rand()-1)*angular_range
lon_goal = lon0 + (2*np.random.rand()-1)*angular_range
alt_goal = -TARGET_ALTITUDE
global_goal_pose = np.array([lon_goal, lat_goal, alt_goal])
goal_ne = global_to_local(global_goal_pose, global_home_pose)
goal_x, goal_y = int(goal_ne[0])-north_offset, int(goal_ne[1])-east_offset
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
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

Define actions and weights for them:
```python
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```
Check whether the actions are valid:
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
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

```python
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

# We're using collinearity here, but you could use Bresenham as well!
def prune_path(path, eps=1e-6):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
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

### Execute the flight
#### 1. Does it work?
It works!
  
# Extra Challenges: PRM

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.



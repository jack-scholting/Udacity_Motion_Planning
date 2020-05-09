# Project: 3D Motion Planning

![Final Solution Clip](./misc/final_solution.gif)

## Introduction

This is the "3D Motion Planning" project in the Udacity Flying Car NanoDegree program. It is one of the three major projects in the program.

One of the core problems of making Flying Cars (autonomous flying vehicles), is motion planning. These flying cars will need to know how to navigate our world, given a starting point (passenger pickup location) and ending point (passenger destination). You can't get from A to B in the real world by flying a straight line between them at a fixed altitude. Instead, the flying car will have to avoid obstacles and maybe fly some pre-determined routes.

So 3D Motion Planning involves the following things:

- Represent the world - discretize the world into a grid/graph from a map.
- Define start and goal positions - coordinate systems.
- Use a search algorithm and waypoint simplification algorithm to find an efficient path through the obstacles to the goal.

The solution discussed in this write-up discretizes the world into a graph, finds a path using the A* algorithm, and performs waypoint pruning using collinearity testing.

In order to make this project feasible within the time constraints of the nanodegree, some resources are provided:

- A simulator is provided (<https://github.com/udacity/FCND-Simulator-Releases>)
- A python environment is provided (<https://github.com/udacity/FCND-Term1-Starter-Kit>)
- A Udacidrone drone platform is provided with documentation (https://udacity.github.io/udacidrone/docs/welcome.html)
- Starter code is provided (<https://github.com/udacity/FCND-Motion-Planning>)

The following simplifications are made for this project:

- We consider the map (colliders.csv) perfect without errors or omissions.
- We assume our sensors (position) provide perfect information.
- We assume our flying car is the only moving object in the environment.
- We assume there are no environmental disturbances, such as gusts of wind.

This project could be extended to eliminate some of the above assumptions. The final section of this write-up covers some possible extensions.

## Rubric Points

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

The rubric can be found here: <https://review.udacity.com/#!/rubrics/1534/view>

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf  

This document is the Writeup / README. Below I describe how I addressed each rubric point.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

Note: The starter code can be found here: <https://github.com/udacity/FCND-Motion-Planning>. The "Backyard Flyer" solution is included in the starter code repository as well. The "Backyard Flyer" was the introductory project in this NanoDegree.

The motion planning starter code can be thought of as the "Backyard Flyer" code, with one major modification - the four static "box" waypoints are replaced by a more flexible **planning** step that determines the waypoints.

To add this planning step, first a "PLANNING" state was introduced. (see `States(Enum)`)

Over the course of a flight, this "PLANNING" state takes place after the "ARMING" state, but before the "TAKEOFF" state. (see `state_callback()`). So the normal sequence of states is as follows:

MANUAL -> ARMING -> [PLANNING] -> TAKEOFF -> WAYPOINT -> LANDING -> DISARMING -> MANUAL

The ultimate goal/output of the planning step is a set of waypoints. (see `self.waypoints`)

The planning step is implemented by the `plan_path()` function. This function ties together all the utilities found in the `planning_utils.py` file.

The main two utilities provided in `planning_utils.py` are `create_grid()` and `a_star()`. The other functions and definitions in `planning_utils.py` exist simply to support those two main utilities. The utility `create_grid()` takes the colliders.csv data and builds us a representation of the world, including freespace and obstacles, that the `a_star()` utility can use to find an efficient path (or set of waypoints) from the start to the goal positions.

Note: For a more detailed comparison between the "Backyard Flyer" and the motion planning starter code, see the included "Comparison_Report.html".

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

The starter code assumes the home position is where the drone first initializes, but in reality the code should be able to start planning from anywhere.

I added the following code in `plan_path()` to get the map center lat/lon from the csv and set it as the global home:

```python
# Note: The "Configuration Space Exercise" says the first line of 
# colliders.csv will be the latitude and longitude of the center of the
# map.
# Note: The following code makes the assumption that the first line of 
# colliders.csv has the following format:
#    lat0 37.792480, lon0 -122.397450
with open("colliders.csv") as f:
    first_line = f.readlines()[0]
    lat_str, lon_str = first_line.split(",")
    lat_float = np.float64(lat_str.split(" ")[1])
    lon_float = np.float64(lon_str.split(" ")[2])

# Per the assignment, the global home should be set to the map center.
# From the documentation:
# "Set the GPS home position for the drone. This changes the origin point
#  of the local NED frame and therefore adjusts the local position information."
self.set_home_position(lon_float, lat_float, 0)
```

Note: If code above were to be used in a more serious application, I would not make assumptions about the .csv format. Instead I would make it more robust and include error handling.

#### 2. Set your current local position

The starter code assumes the drone takes off from the map center, but the code should allow for taking off anywhere.

I added the following code in `plan_path()` to find the local position.

```python
# Determine the local position.
local_pos = global_to_local(self.global_position, self.global_home)
```

The `global_to_local()` function was already implemented in `udacidrone.frame_utils`. The parameter `self.global_home` was determined in the previous step. The parameter `self.global_position` was available to use per the API.

Note: It doesn't appear that I needed to do this "Set your current local position" step. Per the API documentation, `self.local_position` is available and provides the same information.

#### 3. Set grid start position from local position

The starter code assumes planning is always done from the map center, but the code should perform planning from the current local position.

I replaced the following starter code:

```python
grid_start = (-north_offset, -east_offset)
```

With the following code to start planning from the local position:

```python
# Define starting point on the grid as the current position.
grid_start = (int(local_pos[0]-north_offset), int(local_pos[1]-east_offset))
```

The `north_offset` and `east_offset` were provided by the `create_grid()` function. The offset needs to be applied since the grid (0,0) point is at the bottom left corner of the map, but the local coordinates (0,0) point is at the center of the map.

#### 4. Set grid goal position from geodetic coords

The starter code hardcoded the goal position as some location 10 m north and 10 m east of map center. The code should allow the goal to be set to any arbitrary position on the grid given geodetic coordinates (latitude, longitude).

I replaced the following starter code:

```python
# Set goal as some arbitrary position on the grid
grid_goal = (-north_offset + 10, -east_offset + 10)
```

With the following code:

```python
# Define a goal point.
global_goal = GOAL_POSITION + [self._altitude]
local_goal = global_to_local(global_goal, self.global_home)
grid_goal = (int(local_goal[0]-north_offset), int(local_goal[1]-east_offset))
```

The assignment did not specify how the code should get the global goal position, so I created a `GOAL_POSITION` global that I could set to different test values. Below is an example:

```python
EXTREME_NORTHWEST = [-122.401004, 37.796592]

# Goal position used during planning.
GOAL_POSITION = EXTREME_NORTHWEST
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

The starter code included an A* implementation, but the only valid actions were up/down/left/right. This resulted in a very clunky looking flight plan. The code should allow for diagonal actions.

I added the following code in `class Action(Enum)` to define the new diagonal actions and their cost:

```python
NORTHEAST = (-1, 1, np.sqrt(2))
NORTHWEST = (-1, -1, np.sqrt(2))
SOUTHEAST = (1, 1, np.sqrt(2))
SOUTHWEST = (1, -1, np.sqrt(2))
```

I add the following code in `def valid_actions(grid, current_node):` to remove diagonal actions that are invalid. By invalid, I mean outside the edges of the map, or on an obstacle.

```python
if ((x-1 < 0) or 
    (y+1 > m) or
    (grid[x-1, y+1] == 1)):
    valid_actions.remove(Action.NORTHEAST)
if ((x-1 < 0) or 
    (y-1 < 0) or
    (grid[x-1, y-1] == 1)):
    valid_actions.remove(Action.NORTHWEST)
if ((x+1 > n) or 
    (y+1 > m) or
    (grid[x+1, y+1] == 1)):
    valid_actions.remove(Action.SOUTHEAST)
if ((x+1 > n) or 
    (y-1 < 0) or
    (grid[x+1, y-1] == 1)):
    valid_actions.remove(Action.SOUTHWEST)
```

These modifications made a big difference in the flight plan. Below is a clip of what the flight plan looked like before the modifications:

![Before A* Mods](./misc/before_a_star_diag.gif)

Below is a clip of what the flight plan looked like after the modifications.

![After A* Mods](./misc/after_a_star_diag.gif)

#### 6. Cull waypoints

The starter code did not provide any path pruning. Without path pruning, a waypoint will exist for every grid square, even if the waypoints are all in a line. See the previous section for what that looks like. Instead, the code should produce the minimal number of waypoints needed to get from the start to the goal.

The lectures introduced several path pruning methods. I selected "collinearity" testing for my solution.

I added the following code to `plan_path()`:

```python
# Prune path to minimize number of waypoints
print("Path before pruning: {}".format(path))
pruned_path = prune_path(path)
print("Path after pruning: {}".format(pruned_path))
```

I add the following code to the planning utilities to implement the path pruning. See the comments in the code below for more detailed information:

```python
def point(p):
    """
    Converts an (x,y) tuple to an [x,y,z] array.

    This function was pulled from section 3.9 "Putting it Together Exercise".
    """
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    """
    Checks if three points are all in a line.

    This function was pulled from section 3.9 "Putting it Together Exercise".
    """
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    """
    Removes points in the path that are collinear.

    This function was pulled from section 3.9 "Putting it Together Exercise".
    """
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
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
```

Below is a clip of what the flight path looked like after introducing path pruning:

![After Pruning](./misc/after_pruning.gif)

### Execute the flight

#### 1. Does it work?

Yes, it works. Please see the code and gifs.

I verified my solution by flying a series of goal points around the map, one after the other, without resetting. No errors or crashes occurred, the drone avoided all obstacles, and it appeared to produce a reasonable and efficient flight plath each time.

---

## Additional Modifications

The starter code needed to be modified in a few ways that weren't mentioned in the project assignment. Those modifications are discussed here:

### A* Heuristic Speed

The starter code provided an A* that was Euclidean distance. It used the following code:

```python
return np.linalg.norm(np.array(position) - np.array(goal_position))
```

For some reason, the above code was extremely slow on both my local PC, and the provided Udacity VM. It took several minutes to perform planning, and by the time the solution was produced the connection with the simulator was already timed out.

I replaced the provided code with the more direct implementation of Euclidean distance and it was orders of magnitude faster.

```python
return np.sqrt((goal_position[0]-position[0])**2 + (goal_position[1]-position[1])**2)
```

### A* Robustness

When I first started creating goal positions to test my solution, I was using google maps and for some reason my solution would take 10 minutes to tell me that it could not find a path. It was frustrating because it took a long time, and I thought my goal position was a valid one.

To address this, I added a sanity check in A* to immediately report if the goal position was invalid (i.e. it was inside an obstacle).

```python
# Perform a sanity check that the goal is in a free space.
if grid[goal] == 1:
    print('**********************')
    print('Invalid goal position!')
    print('**********************') 
    return path[::-1], path_cost
```

### Planning Robustness

The starter code simply crashes with a python exception if the drone cannot find a valid planning path. I thought that was a little ungraceful, so I added some code in `plan_path()` to transition back to a safe state if the code could not find a planning path.

```python
if not path:
    # Gracefully handle the case where we cannot find a path during planning.
    self.disarming_transition()
else:
```

---

## Extra Challenges

This solution is designed to ascend to a particular altitude, fly a path at that fixed altitude, at right-angled trajectories, and then land vertically. However, it could be extended using the information provided in the Udacity lectures. Below are some of those concepts:

- Deadbands - to allow smooth transitions through waypoints. The deadband could be a function of velocity by modifying the logic in the local_position_callback() function.
- Probabisitic Roadmap - break free from the fixed altitude and perform efficient full 3D planning.
- Receding Horizon Planning - perform more detailed local planning to deal with errors in the map.
- Automatic Replanning - constantly checking if the path to next waypoint is feasible, or if a new plan needs to be created.
- Add heading commands to waypoints - to allow creation of more interesting flight paths.
- Implement a vehicle model that takes dynamic constraints into account.
- Potential Field based planning
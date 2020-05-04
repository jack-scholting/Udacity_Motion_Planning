# Project: 3D Motion Planning

<!-- ![Quad Image](./misc/enroute.png) -->

## Introduction

This is the "3D Motion Planning" project in the Udacity Flying Car NanoDegree program. It is one of the 3 major projects in the program.

One of the core problems of making Flying Cars (autonomous flying vehicles), is motion planning. These flying cars will need to know how to navigate our world, given a starting point (passenger pickup location) and ending point (passenger destionation). You can't get from A to B in the real world by flying a straight line between them at a fixed altitude. Instead, the flying car will have to avoid obstacles and maybe fly some pre-determined routes.

So 3D Motion Planning involves the following things:

- Represent the world - discretize the world into a grid/graph from a map.
- Define start and goal positions - coordinate systems.
- Use a search algorithm and waypoint simplification algorithm to find an efficient path through the obstactles to our goal.

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
- There is no requirement for how accurate he landing must be on the goal location.

TODO: - finish this set of assumptions. Maybe add some mitigations.

TODO: Underlying infrastructure built so we can issue high level commands.
This project is set up to ascend to a particular altitude, fly a path at that fixed altitude, at right-angled trajectories, and then land vertically.

TODO: Design decisions - grid/graph, search alg, waypoint simplication alg, etc.

TODO: Make sure to add 5+m safety margin to account for "mismatch between the coliders map and the actual buildings in the simulator scene."

TODO: Video of starter code. Video of final code. Before diagonals. Before collinearity test.

## Verification

1. Set some different start locations.
1. Set some different goal locations.

## Required Steps for a Passing Submission

1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

The motion planning starter code can be thought of as the "Backyard Flyer" code, with one major modification - the four static "box" waypoints are replaced by a more flexible **planning** step.

To add this planning step, first a "PLANNING" state was introduced. (see definition "States(Enum)")

Over the course of a flight, this "PLANNING" state takes place after the "ARMING" state, but before the "TAKEOFF" state. (see "state_callback()")

The ultimate goal/output of the planning step is a set of waypoints. (see "self.waypoints")

The planning step is implemented by the "plan_path()" function. This function ties together all the utilities found in the "planning_utils.py" file.

The main two utilities provided in "planning_utils.py" are "create_grid()" and "a_star()". The other functions and definitions in "planning_utils.py" exist simply to support those two main utilities. The utility "create_grid()" takes the colliders.csv data and builds us a representation of the world, including freespace and obstacles, that the "a_star()" utility can use to find an efficient path (or set of waypoints) from the start to the goal positions.

Note: For a more detailed comparison between the "Backyard Flyer" and the motion planning starter code, see the included "Comparison_Report.pdf".

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

The starter code assumes the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere.

Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

TODO: Why does the global home position matter?

TODO: Positions/Terminology:

- Current Position - can either be global or local. You find global with a sensor like a GPS. You find local by knowing where the local home (0,0) is located globally.
- Starting Position - this is where you start planning I guess. Seems like it should always be your current position.
- Goal Position - arbitrarily set by anyone. Can be global or local coordinates.

#### 2. Set your current local position

The starter code assumes the drone takes off from the map center, but it will need to be able to takeoff from anywhere.

Retrieve your current position in geodetic coordinates from self._latitude, self._longitude, and self._altitude. Then use the utility function global_to_local() to convert to local position(using self.global_home as well, which you just set).

Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

#### 3. Set grid start position from local position

TODO: 

- So grid (0,0) is lower left corner of the grid.
- But the coordinates in colliders.csv have the middle of the map as (0,0), like a home position.
- My drone local position is based off the global home being the center of the map.
- This means if I have a local position, and want to get the correct grid location, I need to apply an offset to move my center point from the center of the map to the bottom left corner.
- This offset is most southern and most western point (min north, east).

The starter code assumes planning is always done from the map center. Instead, it should be done from the current local position.

This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords

The starter code hardcoded the goal position as some location 10m north and 10m east of map center. This code needed to be modified to allow the goal to be set to any arbitrary position on the grid given geodetic coordinates (latitude, longitude).

This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

TODO: how should the goal be set? Just a constant in the file?

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints

For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

### Execute the flight

#### 1. Does it work?

It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points
  
---

## Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.

Challenge Ideas:

- Probabisitic Roadmap
- Receding Horizon Planning
- Automatic Replanning - constantly checking if path to next waypoint is feasible, or if new plan needs to be developed.
- "Play around with a dynamical model for the vehicle"
- Deadzones - to allow smooth transitions through waypoints. "even try making deadbands a function of velocity. To do this, you can simply modify the logic in the local_position_callback() function."
- Add heading commands to your waypoints - see the github README for details`
- Potential Fields

---

## Example Markdown

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

from enum import Enum
from queue import PriorityQueue
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    print("Grid Info: N min {}, N max {}, E min {}, E max {}, N size {}, E size {}".format(north_min, north_max, east_min, east_max, north_size, east_size))

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTHEAST = (-1, 1, np.sqrt(2))
    NORTHWEST = (-1, -1, np.sqrt(2))
    SOUTHEAST = (1, 1, np.sqrt(2))
    SOUTHWEST = (1, -1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
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

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    # Perform a sanity check that the goal is in a free space.
    if grid[goal] == 1:
        print('**********************')
        print('Invalid goal position!')
        print('**********************') 
        return path[::-1], path_cost
    
    else:
        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:              
                current_cost = branch[current_node][0]
                
            if current_node == goal:        
                print('Found a path.')
                found = True
                break
            else:
                # print("Current cost: {}".format(current_cost))
                for action in valid_actions(grid, current_node):
                    # get the tuple representation
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)
                    
                    if next_node not in visited:                
                        visited.add(next_node)               
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('*********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def heuristic(position, goal_position):
    # Note: This is Euclidean distance, which works for both gird and 
    # graph based discretizations of the environment.
    return np.sqrt((goal_position[0]-position[0])**2 + (goal_position[1]-position[1])**2)

    # Note: The implementation provided in the starter code, shown below, 
    # is very slow. It takes multiple minutes to run A* with this heuristic.
    #return np.linalg.norm(np.array(position) - np.array(goal_position))


#TODO: Document these functions. What version of collinearity? Where pulled from?
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    """
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

# def prune_path(path):
#     if path is not None:
#         pruned_path = []

#         # At a selected point, grab the next two. If they are all collinear,
#         # then remove the middle point. If they aren't increment to the 
#         # next point.
#         start = 0
#         mid = 1
#         end = 2
        
#         end_collinear = False
        
#         # Keep pruning until our 3rd point is the last in the path.
#         while(end < len(path)-1):
    
#             pruned_path.append(path[start])
            
#             while (collinearity_check(point(path[start]), point(path[mid]), point(path[end]))):
                
#                 if (end < len(path)-1):
#                     # Move to the next point to test.
#                     mid  = end
#                     end += 1
#                 else:
#                     # end is the last point, and everything is collinear.
#                     break

#             if (end < len(path)-1):
#                 # Scoot all points
#                 start = mid
#                 mid   = end
#                 end  += 1
#             else:
#                 # Reached the end, and last points not collinear. Add mid.
#                 pruned_path.append(path[mid])
                    
#         # Always add the last point (destination)
#         pruned_path.append(path[len(path)-1])
            
#     else:
#         pruned_path = path
        
#     return pruned_path
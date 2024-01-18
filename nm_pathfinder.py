from math import inf, sqrt, pow
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    #loop through all boxes to find the start and end boxes
    start = None 
    end = None
    for box in mesh['boxes']:
        if bounded(source_point, box): start = box
        if bounded(destination_point, box): end = box
    if start is None or end is None:
        print("Source or destination points missing from mesh data", flush="true")


    #bfs looking for a path connecting source_point and destination_point
    # to_visit = []
    # to_visit.append(start)
    # visited = []
    # parent = {} #keeps track of parent of each node
    # i = 10000
    # while len(to_visit) > 0 and i > 0:
    #     curr = to_visit.pop(0)
    #     visited.append(curr)
    #     #if reached end box, return path
    #     if bounded(destination_point, curr):
    #         return (backtrace(end, start, parent), visited)
    #     for neighbor in neighbors(curr, mesh):
    #         #if neighbors have not been visited, add to to_visit
    #         if neighbor not in visited:
    #             to_visit.append(neighbor)
    #             parent[neighbor] = curr
    # print("Valid path does not exist between source and destination points")
    # return ([], visited)


    #modifying Dijkstra's into A*
    #uses modified code from Dijkstra_forward_search.py from this assignment
    # paths = {start: []}          # maps cells to previous cells on path
    # pathcosts = {start: 0}       # maps cells to their pathcosts (found so far)
    # queue = []
    # visited = []
    # detail_points = {start: source_point, end: destination_point}
    # heappush(queue, (0, start))  # maintain a priority queue of cells
    
    # while queue:
    #     priority, cell = heappop(queue)
    #     visited.append(cell)
    #     if cell == end:
    #         return (path_to_cell(start, cell, paths, detail_points), visited)
        
    #     # investigate children
    #     for child in neighbors(cell, mesh):
    #         # calculate cost along this path to child
    #         #distance to current + distance to child
    #         cost_to_child = pathcosts[cell] + distance(detail_points[cell], clamp(detail_points[cell], child))
    #         if child not in pathcosts or cost_to_child < pathcosts[child]:
    #             pathcosts[child] = cost_to_child            # update the cost
    #             paths[child] = cell                         # set the backpointer
    #             detail_points[child] = clamp(detail_points[cell], child)
    #             heappush(queue, (cost_to_child + distance(detail_points[child], destination_point), child)) # put the child on the priority queue


    #modifying A* into bidirectional A*
    #uses modified code from Dijkstra_forward_search.py from this assignment
    
    if start is None or end is None:
        return [], []

    paths_forward = {start: []}          # maps cells to previous cells on path
    paths_backward = {end: []}
    pathcosts_forward = {start: 0}       # maps cells to their pathcosts (found so far)
    pathcosts_backward = {end: 0}
    queue = []
    visited = []
    detail_points_forward = {start: source_point, end: destination_point}
    detail_points_backward = {start: source_point, end: destination_point}
    heappush(queue, (0, start, end))  # maintain a priority queue of cells
    heappush(queue, (0, end, start))
    
    while queue:
        priority, cell, goal = heappop(queue)
        visited.append(cell)

        #using the other paths lists, and checking termination condition
        #majority of the code modified from A* to bidirectional A*
        if goal == start:
            if cell in paths_forward.keys():
                return (path_to_cell_bidirectional(start, end, cell, "forward", paths_forward, paths_backward, detail_points_forward, detail_points_backward), visited)
            paths = paths_backward
            pathcosts = pathcosts_backward
            detail_points = detail_points_backward
        elif goal == end:
            if cell in paths_backward.keys():
                return (path_to_cell_bidirectional(start, end, cell, "backward", paths_forward, paths_backward, detail_points_forward, detail_points_backward), visited)
            paths = paths_forward
            pathcosts = pathcosts_forward
            detail_points = detail_points_forward

        # investigate children
        for child in neighbors(cell, mesh):
            # calculate cost along this path to child
            #distance to current + distance to child
            cost_to_child = pathcosts[cell] + distance(detail_points[cell], clamp(detail_points[cell], child))
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child            # update the cost
                paths[child] = cell                         # set the backpointer
                detail_points[child] = clamp(detail_points[cell], child)
                heappush(queue, (cost_to_child + distance(detail_points[child], goal), child, goal)) # put the child on the priority queue

#returns path from start to end for bidirectional A* implementation
def path_to_cell_bidirectional(start, end, termination_cell, longer_path, paths_forward, paths_backward, detail_points_forward, detail_points_backward):
    if end == [] or start == end:
        return []
    path = []
    if longer_path == "forward":
        path.append(detail_points_forward[termination_cell])

    curr = termination_cell
    while curr is not start:
        path.append(detail_points_forward[paths_forward[curr]])
        curr = paths_forward[curr]

    path.reverse()

    path.append(detail_points_forward[termination_cell])
    path.append(detail_points_backward[termination_cell])

    if longer_path == "backward":
         path.append(detail_points_backward[termination_cell])

    curr = termination_cell
    while curr is not end:
        path.append(detail_points_backward[paths_backward[curr]])
        curr = paths_backward[curr]
    

    return path


#returns path to a given cell for the A* implementation
def path_to_cell(start, cell, paths, detail_points):
    if cell == []:
        return []
    curr = cell
    path = []
    path.append(center(cell))
    while curr is not start:
        path.append(detail_points[paths[curr]])
        curr = paths[curr]
    path.reverse()
    return path

#clamp the coords of point a to the bounds of boxB
def clamp(point, box):
    return (min(box[1], max(box[0], point[0])), min(box[3], max(box[2], point[1])))

#returns "neighbor" boxes that pathfinding can visit from a given box
def neighbors(centerBox, mesh):
    return mesh['adj'][centerBox] # square grid adjacency heuristic

#returns "distance" between two points of form (x, y)
def distance(a, b):
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2)) #manhattan distance heuristic

# given a start point, end point and dict of parents to each node, return a path from start to end
def backtrace(start, end, parent):
    curr = start
    path = []
    while curr is not end:
        path.append(center(curr))
        curr = parent[curr]
    path.append(center(curr))
    path.reverse()
    return path

#returns if a given point is bounded by a given box
def bounded(point, box):
    return point[0] > box[0] and point[0] < box[1] and point[1] > box[2] and point[1] < box[3]

#returns center coordinates of a given box
def center(box):
    return ((box[0] + box[1]) / 2, (box[2] + box[3]) / 2)






#first implementations before 1/17



# def heuristic(a, b):
#     return abs(a[0] - b[0]) + abs(a[1] - b[1])
#     # a is x axis, b is y axis
#     # absolute value of both, Manhatten distance

# def find_path(source_point, destination_point, mesh):
#     start = (int(source_point[0]), int(source_point[1]))
#     goal = (int(destination_point[0]), int(destination_point[1]))

#     # Priority queue for open set in A* algorithm
#     open_set = []
#     heapq.heappush(open_set, (0, start))

#     # A path (list of points) from source_point to destination_point if exists
#     path = []

#     # dictionary to store the cost to reach each point
#     reach_cost = {start: 0}

#     # dictionary to store the parent of each point
#     parents = {start: None}

#     # list to store explored boxes
#     reached_boxes = {}

#     # main loop while prio queue is open
#     while open_set:
#         # current is the front of queue
#         current = heapq.heappop(open_set)[1]

#         # goal is found, current is at the goal
#         if current == goal:
#             # Reconstruct the path that was taken
#             path = []
#             while current:
#                 # reverse the path taken by using parents
#                 path.append(current)
#                 current = parents[current]
#             # reverse that path to show start to goal, instead of goal to start
#             path.reverse()
#             return path, reached_boxes.keys()

#         for neighbor in mesh.get(current, []):
#             # traverse loop
#             neighbor = tuple(neighbor)
#             # neighbor is not reached yet or the reach progresses through path, see reading for more info
#             if neighbor not in reach_cost or reach_cost[current] + 1 < reach_cost[neighbor]:
#                 #&& if outline != "black": idk how to code that
#                 # assign to reach_cost and progress
#                 reach_cost[neighbor] = reach_cost[current] + 1
#                 priority = reach_cost[neighbor] + heuristic(neighbor, goal)
#                 heapq.heappush(open_set, (priority, neighbor))
#                 parents[neighbor] = current


#         # Record the box that has been reached
#         box = current
#         if box not in reached_boxes:
#             reached_boxes[box] = True

#     # If no path is found
#     return None, reached_boxes.keys()
#     print("no path found")
     











    # paths = {source_point: []}          # maps cells to previous cells on path
    # pathcosts = {source_point: 0}       # maps cells to their pathcosts (found so far)
    # queue = []
    # heappush(queue, (0, source_point))  # maintain a priority queue of cells
    
    # while queue:
    #     priority, cell = heappop(queue)
    #     if cell == destination:
    #         return path_to_cell(cell, paths)
        
    #     # investigate children
    #     for (child, step_cost) in adj(graph, cell):
    #         # calculate cost along this path to child
    #         cost_to_child = priority + transition_cost(graph, cell, child)
    #         if child not in pathcosts or cost_to_child < pathcosts[child]:
    #             pathcosts[child] = cost_to_child            # update the cost
    #             paths[child] = cell                         # set the backpointer
    #             heappush(queue, (cost_to_child, child))     # put the child on the priority queue
            
    # return False









# # # me trying to implement forward search to bidirectional
# from math import sqrt
# from heapq import heappop, heappush

# def find_path(source_point, destination_point, mesh):
#     path = []
#     boxes = {}

#     forward_paths = {source_point: []}
#     forward_pathcosts = {source_point: 0}
#     backward_paths = {destination_point: []}
#     backward_pathcosts = {destination_point: 0}

#     queue = []
#     heappush(queue, (0 + heuristic(source_point, destination_point), source_point, 'forward'))
#     heappush(queue, (0 + heuristic(destination_point, source_point), destination_point, 'backward'))

#     intersection_point = None

#     while queue:
#         priority, cell, goal = heappop(queue)

#         if goal == 'forward':
#             current_paths = forward_paths
#             current_pathcosts = forward_pathcosts
#             other_paths = backward_paths
#             other_pathcosts = backward_pathcosts
#         else:
#             current_paths = backward_paths
#             current_pathcosts = backward_pathcosts
#             other_paths = forward_paths
#             other_pathcosts = forward_pathcosts

#         if cell in other_paths:
#             intersection_point = cell
#             break

#         for (child, step_cost) in navigation_edges(mesh, cell):
#             cost_to_child = current_pathcosts[cell] + step_cost
#             if child not in current_pathcosts or cost_to_child < current_pathcosts[child]:
#                 current_pathcosts[child] = cost_to_child
#                 current_paths[child] = cell
#                 heappush(queue, (cost_to_child + heuristic(child, destination_point), child, goal))

#     if intersection_point:
#         # Build the path segments from both directions
#         forward_segment = path_to_cell(intersection_point, forward_paths)
#         backward_segment = path_to_cell(intersection_point, backward_paths)[::-1]  # Reverse the backward path

#         # Combine the two segments
#         path = forward_segment + backward_segment[1:]  # Exclude the intersection point
#         explored_boxes = set(forward_paths.keys()).union(backward_paths.keys())
#         return path, explored_boxes

#     return []

# def path_to_cell(cell, paths):
#     if cell == []:
#         return []
#     return path_to_cell(paths[cell], paths) + [cell]

# def heuristic(cell, destination):
#     # Euclidean distance heuristic
#     return sqrt((destination[0] - cell[0]) ** 2 + (destination[1] - cell[1]) ** 2)

# def navigation_edges(mesh, cell):
#     res = []
#     for delta in [(x, y) for x in [-1, 0, 1] for y in [-1, 0, 1] if not (x == 0 and y == 0)]:
#         new = (cell[0] + delta[0], cell[1] + delta[1])
#         if new in mesh['spaces']:
#             res.append((new, transition_cost(mesh, new, cell)))
#     return res

# def transition_cost(mesh, cell, cell2):
#     distance = sqrt((cell2[0] - cell[0]) ** 2 + (cell2[1] - cell[1]) ** 2)
#     average_cost = (mesh['spaces'][cell] + mesh['spaces'][cell2]) / 2
#     return distance * average_cost





# def find_path (source_point, destination_point, mesh):

#     """
#     Searches for a path from source_point to destination_point through the mesh

#     Args:
#         source_point: starting point of the pathfinder
#         destination_point: the ultimate goal the pathfinder must reach
#         mesh: pathway constraints the path adheres to

#     Returns:

#         A path (list of points) from source_point to destination_point if exists
#         A list of boxes explored by the algorithm
#     """

#     path = []
#     boxes = {}

#     return path, boxes.keys()




# import heapq

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





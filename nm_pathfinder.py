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

    path = []
    boxes = {}

    return path, boxes.keys()




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


import heapq
import matplotlib.pyplot as plt
import numpy as np

# Define the maze (0 is open, 1 is an obstacle)
maze = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]

# Define start and end points
start = (0, 0)
end = (0, 4)

# Directions for moving in the maze (right, down, left, up)
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

# Function to check if a position is valid
def is_valid_pos(maze, pos):

    x, y = pos

    # 0 <= x < len(maze): This checks if x is within the left and right boundaries of the maze. 

    #0 <= y < len(maze[0]): This checks if y is within the top and bottom boundaries of the maze.

    #maze[x][y] == 0: This checks if the position (x, y) is an open space. If it's 0, it's open. If it's 1, it's a wall.

    return 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y] == 0

# Heuristic function to estimate distance to end (Manhattan distance)
def heuristic(a, b):

    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Hybrid A* algorithm
def hybrid_a_star(maze, start, end):
    heap = []
    heapq.heappush(heap, (0, start))

    came_from = {start: None} # Dictionary to store the path. The key is the current position and the value is the previous position. It tells us the previous position (where we came from) for each position we've visited
    # Look up (2, 1) in came_from to find where we came from: (2, 0).

    cost_so_far = {start: 0} # Dictionary to store the cost

    while heap:
        current_cost, current = heapq.heappop(heap)

        if current == end:
            break

        for direction in directions:
            next_pos = (current[0] + direction[0], current[1] + direction[1])
            # print(next_pos)
            if is_valid_pos(maze, next_pos):
                # print("---------> valid <---------")
                new_cost = cost_so_far[current] + 1  # Each move has a cost of 1
                print(new_cost)
                print(next_pos)
                print(current)
                print(cost_so_far)
                
                print("\033[93mPriority value: " + str(came_from) + "\033[0m")

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    print("\033[92m----> valid <----\033[0m")

                    print("\033[94m Next position: " + str(next_pos) + "\033[0m")

                    cost_so_far[next_pos] = new_cost # Update the cost 

                    priority = new_cost + heuristic(end, next_pos)  # Calculate the priority value: cost + heuristic value to know witch value is the best or the shortests
                    print("\033[94mPriority value: " + str(priority) + "\033[0m")

                    heapq.heappush(heap, (priority, next_pos))

                    print("\033[94mPriority value: " + str(heap) + "\033[0m")

                    came_from[next_pos] = current
        print("\033[91m ------------------------------> <--------------------------\033[0m")



    # Reconstruct path
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()


# Initial State:
# End: (0, 4)
# came_from: (0, 4) -> (0, 3)
# Path so far: [(0, 4)]
# Step 1:
# Current: (0, 4)
# Look up: came_from[(0, 4)] -> (0, 3)
# Update path: [(0, 4), (0, 3)]
# Step 2:
# Current: (0, 3)
# Look up: came_from[(0, 3)] -> (0, 2)
# Update path: [(0, 4), (0, 3), (0, 2)]
# Step 3:
# Current: (0, 2)
# Look up: came_from[(0, 2)] -> (1, 2)
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2)]
# Step 4:
# Current: (1, 2)
# Look up: came_from[(1, 2)] -> (2, 2)
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2), (2, 2)]
# Step 5:
# Current: (2, 2)
# Look up: came_from[(2, 2)] -> (2, 1)
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2), (2, 2), (2, 1)]
# Step 6:
# Current: (2, 1)
# Look up: came_from[(2, 1)] -> (2, 0)
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2), (2, 2), (2, 1), (2, 0)]
# Step 7:
# Current: (2, 0)
# Look up: came_from[(2, 0)] -> (1, 0)
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2), (2, 2), (2, 1), (2, 0), (1, 0)]
# Step 8:
# Current: (1, 0)
# Look up: came_from[(1, 0)] -> (0, 0)
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2), (2, 2), (2, 1), (2, 0), (1, 0), (0, 0)]
# Step 9:
# Current: (0, 0)
# Look up: came_from[(0, 0)] -> None
# Update path: [(0, 4), (0, 3), (0, 2), (1, 2), (2, 2), (2, 1), (2, 0), (1, 0), (0, 0)]

    print("\033[93mPriority value: " + str(came_from) + "\033[0m")


    return path

# Find the path using Hybrid A*
path = hybrid_a_star(maze, start, end)

# Plot the maze and the path
fig, ax = plt.subplots()
maze_plot = np.array(maze)
ax.imshow(maze_plot, cmap=plt.cm.gray)

# Plot the path
path_x, path_y = zip(*path)
ax.plot(path_y, path_x, marker='o', color='red')

# Start and end points
ax.plot(start[1], start[0], marker='o', color='green', markersize=10)
ax.plot(end[1], end[0], marker='o', color='blue', markersize=10)

plt.show()

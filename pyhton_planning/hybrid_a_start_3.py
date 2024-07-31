import heapq
import matplotlib.pyplot as plt
import numpy as np
import math as m
from math import fmod,pi

class State:
    def __init__(self, x, y, heading) -> None:
        self.x = x
        self.y = y
        self.heading = normalize_angle(heading)
        self.steer = 0
        self.x_index = int(x/GridMap.xy_resolution)
        self.y_index = int(y/GridMap.xy_resolution)
        self.heading_index = int((self.heading+m.pi)/GridMap.heading_resolution)
        self.direction_index = -1
        self.parent = [-1, -1, -1]
        self.move_angle = GridMap.heading_resolution

        def normalize_angle(angle):
            a = fmod(fmod(angle, 2.0*pi) + 2.0*pi, 2.0*pi)
            if a > pi:
                a -= 2.0 *pi
            return a

class VehicleConfig:
    def __init__(self) -> None:
        self.length = 3.5
        self.width = 2
        self.baselink_to_front = 3
        self.baselink_to_rear = self.length - self.baselink_to_front
        self.wheel_base = 2.5
        self.max_front_wheel_angle = 0.6  # rad

# GridMap class definition
class GridMap:
    def __init__(self, world_w, world_h) -> None:
        self.xy_resolution = 1.0
        self.heading_resolution = m.pi / 4
        self.world_width = world_w
        self.world_height = world_h
        self.map_w = int(world_w / self.xy_resolution)
        self.map_h = int(world_h / self.xy_resolution)
        self.headings = int(m.pi * 2 / self.heading_resolution)
        self.default_map = self.generate_map()

    def generate_map(self):
        default_map = np.zeros((self.map_h, self.map_w), dtype=int)
        # Add obstacles for demonstration
        default_map[5:7, 5:15] = 1
        return default_map

# Heuristic function to estimate distance to end (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Hybrid A* algorithm
def hybrid_a_star(maze, start, end):
    heap = []
    heapq.heappush(heap, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up

    while heap:
        current_cost, current = heapq.heappop(heap)

        if current == end:
            break

        for direction in directions:
            next_pos = (current[0] + direction[0], current[1] + direction[1])
            if is_valid_pos(maze, next_pos):
                new_cost = cost_so_far[current] + 1  # Each move has a cost of 1
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(end, next_pos)
                    heapq.heappush(heap, (priority, next_pos))
                    came_from[next_pos] = current

    # Reconstruct path
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = came_from.get(current)  # Use get() to avoid KeyError
        if current is None and path[-1] != start:
            print("Path not found, terminating path reconstruction.")
            return []

    path.reverse()
    return path

# Function to check if a position is valid
def is_valid_pos(maze, pos):
    x, y = pos
    return 0 <= x < maze.shape[1] and 0 <= y < maze.shape[0] and maze[y, x] == 0

# Initialize the grid map and obstacles
grid_map = GridMap(20, 10)

# Define start and end points
start = (0, 0)
end = (10, 9)  # Adjusted to fit within the grid

start_state = State(0.0, 0.0, 0.0)
goal_state = State(10.0, 9.0, 0.0)

# Find the path using Hybrid A*
path = hybrid_a_star(grid_map.default_map, start, end)

# Plot the maze and the path
fig, ax = plt.subplots()
ax.imshow(grid_map.default_map, cmap=plt.cm.gray)

# Plot the path
if path:
    path_x, path_y = zip(*path)
    ax.plot(path_x, path_y, marker='o', color='red')
else:
    print("No path found!")

# Start and end points
ax.plot(start[0], start[1], marker='o', color='green', markersize=10)
ax.plot(end[0], end[1], marker='o', color='blue', markersize=10)

plt.show()

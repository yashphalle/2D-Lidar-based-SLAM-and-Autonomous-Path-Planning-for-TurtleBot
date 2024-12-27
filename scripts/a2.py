import numpy as np
import math
import heapq
import rospy

def calc_cost(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((y2 - y1)**2 + (x2 - x1)**2)

def calc_f(curr, goal, start):
    """Calculate the total cost (f = g + h)."""
    g = calc_cost(curr[0], curr[1], start[0], start[1])
    h = calc_cost(curr[0], curr[1], goal[0], goal[1])
    return g + h

def add_neighbours(grid, curr, goal, start, open_list, closed_set, cost_map, parent_map):
    """Add valid neighbors to the open list."""
    directions = [(-1, 0), (-1, -1), (-1, 1), (0, 1), (0, -1), (1, 0), (1, -1), (1, 1)]
    for dx, dy in directions:
        nx, ny = curr[0] + dx, curr[1] + dy
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and (nx, ny) not in closed_set:
            if grid[nx, ny] != 1:  # Check for obstacles
                new_cost = cost_map[curr] + calc_cost(curr[0], curr[1], nx, ny)
                if (nx, ny) not in cost_map or new_cost < cost_map[(nx, ny)]:
                    cost_map[(nx, ny)] = new_cost
                    parent_map[(nx, ny)] = curr
                    heapq.heappush(open_list, (new_cost + calc_cost(nx, ny, goal[0], goal[1]), (nx, ny)))

def a_star(grid, start, goal):
    """Perform A* search."""
    open_list = []
    heapq.heappush(open_list, (0, start))
    closed_set = set()
    cost_map = {start: 0}
    parent_map = {}

    while open_list:
        _, curr = heapq.heappop(open_list)
        if curr in closed_set:
            continue
        closed_set.add(curr)

        if curr == goal:
            # Reconstruct the path
            path = []
            while curr:
                path.append(curr)
                curr = parent_map.get(curr)
            return path[::-1]

        add_neighbours(grid, curr, goal, start, open_list, closed_set, cost_map, parent_map)

    return None  # No path found

if __name__ == "__main__":
    print("A* Algorithm")
    # grid = np.array([
    #     [0, 0, 0, 0, 0],
    #     [0, 0, 1, 0, 0],
    #     [0, 0, 1, 0, 0],
    #     [0, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0]
    # ])
    # start = (0, 0)
    # goal = (4, 4)
    # path = a_star(grid, start, goal)
    # if path:
    #     print("Path found:", path)
    # else:
    #     print("No path found.")
    rospy.init_node("a-star")
    grid= rospy.Subscriber()
    while(True):


        rospy.spin()


import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, grid_map, resolution=0.05):
        self.grid = np.array(grid_map)
        self.height, self.width = self.grid.shape
        self.resolution = resolution

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def neighbors(self, node):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.height and 0 <= ny < self.width:
                if self.grid[nx][ny] == 0:
                    result.append((nx, ny))
        return result

    def plan(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + self.heuristic(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        return None  # No path found

    def reconstruct_path(self, came_from, start, goal):
        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

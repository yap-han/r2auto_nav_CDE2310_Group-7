# =========================
# map_utils.py (Updated)
# =========================

from scipy.ndimage import binary_dilation
from collections import deque
import random
import numpy as np
import math

def detect_frontiers(
    occupancy_grid,
    width,
    height,
    dilation_iterations=6,        # Suggestion 2: more aggressive dilation
    cluster_distance=2.0,
    min_cluster_size=5,          # Suggestion 1: increase to ignore tiny clusters
    sample_k=3
):
    grid = np.array(occupancy_grid).reshape((height, width))
    free_mask = (grid == 0)
    unknown_mask = (grid == -1)

    n_free = np.count_nonzero(free_mask)
    n_unknown = np.count_nonzero(unknown_mask)
    print(f"[DEBUG detect_frontiers] free={n_free}, unknown={n_unknown}")

    dilated_free = free_mask.copy()
    for _ in range(dilation_iterations):
        dilated_free = binary_dilation(dilated_free)

    frontier_mask = np.logical_and(dilated_free, unknown_mask)
    frontier_coords = list(zip(*np.where(frontier_mask)))
    print(f"[DEBUG detect_frontiers] frontier_coords found = {len(frontier_coords)}")

    if not frontier_coords:
        return []

    clusters = cluster_frontiers(frontier_coords, cluster_distance, width, height, grid)
    clusters = [c for c in clusters if c['size'] >= min_cluster_size]
    print(f"[DEBUG detect_frontiers] cluster count (filtered) = {len(clusters)}")

    for cluster in clusters:
        cluster['samples'] = sample_points_from_cluster(cluster['pixels'], sample_k)

    # Suggestion 4: sort clusters by size descending
    clusters.sort(key=lambda c: c['size'], reverse=True)

    return clusters

def cluster_frontiers(frontier_pixels, cluster_distance, width, height, grid):
    frontier_set = set(frontier_pixels)
    visited = set()
    clusters = []

    def neighbors(r, c):
        for nr in [r-1, r, r+1]:
            for nc in [c-1, c+1]:
                dist = max(abs(nr - r), abs(nc - c))
                if dist <= cluster_distance and (nr, nc) in frontier_set:
                    yield (nr, nc)

    for pix in frontier_pixels:
        if pix in visited:
            continue
        queue = deque([pix])
        visited.add(pix)
        cluster_points = []

        while queue:
            current = queue.popleft()
            cluster_points.append(current)
            cr, cc = current
            for nb in neighbors(cr, cc):
                if nb not in visited:
                    visited.add(nb)
                    queue.append(nb)

        best_r, best_c = select_best_frontier_pixel(cluster_points, grid, width, height)
        clusters.append({
            'y': best_r,
            'x': best_c,
            'size': len(cluster_points),
            'pixels': cluster_points
        })

    return clusters

def select_best_frontier_pixel(cluster_points, grid, width, height):
    best_score = -float('inf')
    best_cell = cluster_points[0]

    for (r, c) in cluster_points:
        unknown_count = 0
        free_count = 0
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                rr = r + dr
                cc = c + dc
                if 0 <= rr < height and 0 <= cc < width:
                    if grid[rr, cc] == -1:
                        unknown_count += 1
                    elif grid[rr, cc] == 0:
                        free_count += 1
        score = unknown_count - 0.5 * free_count
        if score > best_score:
            best_score = score
            best_cell = (r, c)

    return best_cell

def sample_points_from_cluster(cluster_pixels, k=3):
    if not cluster_pixels:
        return []
    return random.sample(cluster_pixels, min(k, len(cluster_pixels)))

# Keep the filter_reachable_clusters function but do not use it
def filter_reachable_clusters(clusters, grid, width, height, robot_x, robot_y, resolution, origin):
    """
    Filter clusters to include those where any pixel is reachable from the robot's position.
    """
    # Convert robot's position to grid coordinates
    robot_grid_x = int((robot_x - origin[0]) / resolution)
    robot_grid_y = int((robot_y - origin[1]) / resolution)

    # Ensure the robot's position is within bounds
    if not (0 <= robot_grid_x < width and 0 <= robot_grid_y < height):
        print("[DEBUG filter_reachable_clusters] Robot position out of bounds.")
        return []

    # Perform flood-fill to find reachable cells
    reachable_mask = np.zeros((height, width), dtype=bool)
    queue = deque([(robot_grid_y, robot_grid_x)])
    reachable_mask[robot_grid_y, robot_grid_x] = True

    while queue:
        y, x = queue.popleft()
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-connectivity
            ny, nx = y + dy, x + dx
            if 0 <= ny < height and 0 <= nx < width and not reachable_mask[ny, nx]:
                if grid[ny, nx] == 0:  # Free cell
                    reachable_mask[ny, nx] = True
                    queue.append((ny, nx))

    # Filter clusters based on reachability of any pixel
    reachable_clusters = []
    for cluster in clusters:
        # Validate pixel coordinates
        valid_pixels = [
            (cx, cy) for cx, cy in cluster["pixels"]
            if 0 <= cx < width and 0 <= cy < height
        ]
        if any(reachable_mask[cy, cx] for cx, cy in valid_pixels):  # Check all valid pixels
            reachable_clusters.append(cluster)

    print(f"[DEBUG filter_reachable_clusters] Reachable clusters: {len(reachable_clusters)} / {len(clusters)}")
    return reachable_clusters
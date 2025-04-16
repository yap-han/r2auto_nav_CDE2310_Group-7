import numpy as np
from scipy.ndimage import binary_dilation

def detect_frontiers(occupancy_grid, width, height):
    """
    Detect frontier regions (boundaries between known free space and unexplored cells)
    Args:
        occupancy_grid: 1D array of occupancy values [-1, 0, 100]
        width: map width
        height: map height

    Returns:
        List of frontier cell (x, y) positions
    """
    grid = np.array(occupancy_grid).reshape((height, width))
    free_mask = (grid == 0)
    unknown_mask = (grid == -1)

    dilated_free = binary_dilation(free_mask)
    frontier_mask = np.logical_and(dilated_free, unknown_mask)

    frontier_coords = list(zip(*np.where(frontier_mask)))  # (y, x) pairs
    return [(x, y) for y, x in frontier_coords]

def project_heat_cells_to_map(robot_pose, heatmap, cell_resolution=0.05, fov_angle=np.pi/3):
    """
    Project local heatmap detections into global map frame based on robot pose.

    Args:
        robot_pose: (x, y, theta) in global frame
        heatmap: 8x8 numpy array of thermal values
        cell_resolution: size of each global map cell in meters
        fov_angle: sensor field of view in radians (default ~60 deg)

    Returns:
        List of projected (x, y) points in global map frame tagged as hot detections
    """
    x, y, theta = robot_pose
    detections = []
    rows, cols = heatmap.shape
    mid_row, mid_col = rows // 2, cols // 2

    for i in range(rows):
        for j in range(cols):
            temp = heatmap[i][j]
            if temp >= 30.0:  # Hardcoded threshold
                dx = (j - mid_col) * cell_resolution
                dy = (mid_row - i) * cell_resolution

                # Rotate based on robot yaw
                gx = x + (dx * np.cos(theta) - dy * np.sin(theta))
                gy = y + (dx * np.sin(theta) + dy * np.cos(theta))
                detections.append((gx, gy))

    return detections

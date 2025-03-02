import numpy as np
import math
from lidar_settings import *
from hallucinate import *

# Raycasting method that uses Bresenham's line algorithim for an occupancy grid
def grid_raycast(goals_array, right_obstacles, left_obstacles):
    # Ray Cast at each odom for every LIDAR value
    scans = []

    for i, goal in enumerate(goals_array):
        # Generate nearby walls
        walls = []  # Will need to recalculate walls with each goal
        createNearbyWalls(walls, i, right_obstacles, left_obstacles)
        wall_array = np.array(walls)

        # Ray Cast
        ranges = [0] * NUM_RAYS
        x_pos = goal[0]
        y_pos = goal[1]
        z_o = goal[2]

        x_map = (x_pos - X_MAP_MIN) // RESOLUTION
        y_map = (y_pos - Y_MAP_MIN) // RESOLUTION

        ray_angle = z_o - HALF_FOV
        for ray in range(NUM_RAYS):
            sin_a = math.sin(ray_angle)
            cos_a = math.cos(ray_angle)

            # Horizontal lines
            if sin_a > 0:
                y_horiz = y_map + 1
                dy = 1
            else:
                y_horiz = y_map - 1
                dy = -1

            depth_horiz = (y_horiz - y_pos) / sin_a
            x_horiz = x_pos + depth_horiz * cos_a

            delta_depth = dy / sin_a
            dx = delta_depth * cos_a

            for i in range(MAX_DEPTH):
                tile_horiz = int(x_horiz), int(y_horiz)
                for segment in walls:
                    if math.dist(tile_horiz, segment[0]) + math.dist(tile_horiz, segment[1]) == math.dist(segment[0], segment[1]):
                        break   # Ray hit a wall

                x_horiz += dx
                y_horiz += dy
                depth_horiz += delta_depth

            # Vertical lines
            if cos_a > 0:
                x_vert = x_map + 1
                dx = 1
            else:
                x_vert = x_map - 1  # Needs to move to tile to the left
                dx = -1
            
            # cos_a = (x_vert - x_pos) / depth_vert
            depth_vert = (x_vert - x_pos) / cos_a   # Eq of cos for hypotenuse
            # sin_a = (y_vert - y_pos) / depth_vert
            y_vert = (sin_a * depth_vert) + y_pos

            # cos_a = dx / delta_depth
            delta_depth = dx / cos_a
            # sin_a = dy / delta_depth
            dy = sin_a * delta_depth

            for i in range(MAX_DEPTH):
                tile_vert = int(x_vert), int(y_vert)
                for segment in wall_array:
                    if math.dist(tile_vert, segment[0]) + math.dist(tile_vert, segment[1]) == math.dist(segment[0], segment[1]):
                        break   # Ray hit a wall

                x_vert += dx
                y_vert += dy
                depth_vert += delta_depth

            if depth_vert < depth_horiz:
                depth = depth_vert
            else:
                depth = depth_horiz

            ranges.append(depth)
            ray_angle += DELTA_ANGLE # Increment to next ray

        scans.append(ranges)
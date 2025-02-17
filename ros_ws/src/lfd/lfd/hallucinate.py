import csv
import os
import sys
import numpy as np
import math
from lidar_settings import *

'''
A script that takes in the vehicles odometry and places fake obstacles around
the robot. Hallucinated LIDAR data can be created for each point of odometry. 
Raycasting is implemented to produce this fake LIDAR data. Final lidar data is
put into CSV or Ros2 Topic.
'''

# TODO: Figure out if final hallucinated LIDAR should go to CSV or topic

def hallucinateLidar(odom_csv):
    local_goals = []
    odom_times = []
    left_obstacles = []
    right_obstacles = []

    goals_scale = 100
    obstacle_offset = 5

    # Create array of local goals from odom csv
    parseOdom(odom_csv, goals_scale, local_goals, odom_times) # Get local goals from csv
    
    # Generate arrays of points for left and right obstacles
    goals_array = np.array(local_goals)
    walls = []

    # TODO: Check if obstacles should be put into grid world or kept in floating points

    generateObstacles(goals_array, obstacle_offset, right_obstacles, left_obstacles)

    # Plot right and left obstacles with local goals
    
    # Ray Cast at each odom for every LIDAR value
    scans = []

    for goal in goals_array:
        ranges = [0] * NUM_RAYS
        x_pos = goal[0]
        y_pos = goal[1]
        z_o = goal[2]   # Orientation in z-axis

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
                for segment in walls:
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


# Go through csv file for odometry: x, y, yaw (just in case) and generate a list
# of local goals every n odometry data
def parseOdom(odom_csv, n, goals, times):
    try:
        with open(odom_csv, mode='r') as csv_file:
            csv_reader = csv.reader(odom_csv)
            for i, row in enumerate(csv_reader):
                if i % n == 0:
                    # Only save every n odom positions as a goal
                    times.append(row[0])    # timestamp for odom
                    goals.append([row[1], row[2], row[6]])  # x, y, yaw

    except FileNotFoundError:
        print("ERROR: File: ", odom_csv," not found\n")


# Go through list of local goals and calculate the position of each left and
# right obstacle for every goal
def generateObstacles(goals, scalar, right_obstacles, left_obstacles):
    for i in goals:
        if i + 1 < (len(goals) - 1):
            # Haven't reached end yet
            point1 = np.array(goals[i][0], goals[i][1])
            point2 = np.array(goals[i + 1][0], goals[i + 1][1])

            vector = point2 - point1
            vector_mag = np.linalg.norm(vector)
            unit_vector = vector / vector_mag   # unit vector = 1 of what each grid is

            right_unit_vector = np.array([unit_vector[1], -1 * unit_vector[0]])
            right_obstacle_point = point1 + (scalar * right_unit_vector)
            left_obstacle_point = point1 + (-1 * scalar * right_unit_vector)

            right_obstacles.append(right_obstacle_point)
            left_obstacles.append(left_obstacle_point)


if __name__ == "__main__":
    # As this is post processing, robot's odometry is in a csv
    
    odom_csv_filename = sys.argv[1] # Give odom csv file in command line
    if os.path.exists(odom_csv_filename):
        hallucinateLidar(odom_csv_filename)
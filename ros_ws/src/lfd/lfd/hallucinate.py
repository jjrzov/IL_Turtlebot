import csv
import os
import sys
import numpy as np
import math
from lidar_settings import *
from plots import *

'''
A script that takes in the vehicles odometry and places fake obstacles around
the robot. Hallucinated LIDAR data can be created for each point of odometry. 
Raycasting is implemented to produce this fake LIDAR data. Final lidar data is
put into CSV or Ros2 Topic.
'''

OBSTACLES_FORWARD = 5
GOALS_SCALE = 100
OBSTACLE_OFFSET = 5

# TODO: Figure out if final hallucinated LIDAR should go to CSV or topic

def hallucinateLidar(odom_csv):
    local_goals = []
    odom_times = []
    left_obstacles = []
    right_obstacles = []

    # Create array of local goals from odom csv
    parseOdom(odom_csv, GOALS_SCALE, local_goals, odom_times) # Get local goals from csv

    # Generate arrays of points for left and right obstacles
    times_array = np.array(odom_times[1:], dtype=np.float32)
    goals_array = np.array(local_goals[1:], dtype=np.float32)

    # TODO: Check if obstacles should be put into grid world or kept in floating points
    
    generateObstacles(goals_array, OBSTACLE_OFFSET, right_obstacles, left_obstacles)
    right_array = np.array(right_obstacles)
    left_array = np.array(left_obstacles)

    # Plot right and left obstacles with local goals
    # print(goals_array[:5])
    plotPath(goals_array)
    plotObstacles(goals_array, right_array, left_array)

    # Ray Cast at each odom for every LIDAR value
    scans = []

    for i, goal in enumerate(goals_array):
        # Generate nearby walls
        walls = []  # Will need to recalculate walls with each goal
        createNearbyWalls(walls, i, right_array, left_array)
        walls_array = np.array(walls)

        # Ray Cast
        ranges = [0] * NUM_RAYS
        x_pos = goal[0]
        y_pos = goal[1]
        z_o = goal[2]

        ray_angle = z_o - HALF_FOV
        for j in range(NUM_RAYS):
            # Reset distances for each ray
            min_distance = MAX_DEPTH
            distance = 0
            
            # Get equation of line for a ray from point at angle
            ray_slope = math.tan(ray_angle)
            ray_intercept = y_pos - (ray_slope * x_pos)

            for segment in walls_array:
                # Get equation for line that goes through both points of a wall
                segment_slope = (segment[1][1] - segment[0][1]) / (segment[1][0] - segment[0][1])   # Rise / Run
                segment_intercept = segment[1][1] - (segment_slope * segment[1][0])

                # Go through x values that would equal an intersection
                for k in range(segment[0][0], segment[1][0], INTERSECTION_STEP):
                    # See if y values for both eqs are equal to each other (within a tolerance)
                    y_ray = (ray_slope * k) + ray_intercept
                    y_segment = (segment_slope * k) + segment_intercept
                    if abs(y_ray) - abs(y_segment) < INTERSECTION_STEP:
                        distance = math.dist([k, y_segment], goal[0:2])
                        break

                # Hopefully never be a point where ray goes through multiple segments, if so choose min
                if distance < min_distance:
                    min_distance = distance
                
            ranges.append(min_distance)
            ray_angle += DELTA_ANGLE # Increment to next ray

        scans.append(ranges)


# Go through csv file for odometry: x, y, yaw (just in case) and generate a list
# of local goals every n odometry data
def parseOdom(odom_csv, n, goals, times):
    try:
        with open(odom_csv, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
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
    for i in range(len(goals)):
        if i < len(goals) - 1:
            # Haven't reached end yet
            point1 = np.array([goals[i][0], goals[i][1]])
            point2 = np.array([goals[i + 1][0], goals[i + 1][1]])

            vector = point2 - point1
            vector_mag = np.linalg.norm(vector)
            unit_vector = vector / vector_mag   # unit vector = 1 of what each grid is

            right_unit_vector = np.array([unit_vector[1], -1 * unit_vector[0]])
            right_obstacle_point = point1 + (scalar * right_unit_vector)
            left_obstacle_point = point1 + (-1 * scalar * right_unit_vector)

            right_obstacles.append(right_obstacle_point)
            left_obstacles.append(left_obstacle_point)


# Create list of walls nearby obstacle. This list is composed of pairs of points
# that compose each segment of the polygon that can be created from the nearby
# obstacles
def createNearbyWalls(walls, index, right_obstacles, left_obstacles):
    makeWalls(walls, index, right_obstacles, left_obstacles, 0)
    
    if (index - 1) < 0:
        walls.append(right_obstacles[index], left_obstacles[index])
    else:
        walls.append(right_obstacles[index - 1], right_obstacles[index])
        walls.append(left_obstacles[index - 1], left_obstacles[index])
        walls.append(right_obstacles[index - 1], left_obstacles[index - 1])


def makeWalls(walls, index, right_obstacles, left_obstacles, count):
    if (index + 1 < len(right_obstacles) - 1 or count < OBSTACLES_FORWARD):
        makeWalls(walls, index + 1, right_obstacles, left_obstacles, count + 1)
        walls.append(right_obstacles[index], right_obstacles[index + 1])
        walls.append(left_obstacles[index], left_obstacles[index + 1])
    else:
        walls.append(right_obstacles[index], left_obstacles[index])

    return
        


if __name__ == "__main__":
    # As this is post processing, robot's odometry is in a csv
    
    odom_csv_filename = sys.argv[1] # Give odom csv file in command line
    if os.path.exists(odom_csv_filename):
        hallucinateLidar(odom_csv_filename)
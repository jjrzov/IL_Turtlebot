import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def plotPath(goals):
    plt.plot(goals[1:, 0], goals[1:, 1], marker='o', linestyle='-', label="Odom Path")

    plt.gca().invert_yaxis()
    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    plt.savefig("Plots/BasicOdomPath.png")

def plotObstacles(goals, right_obstacles, left_obstacles):
    # Plot odom path
    plt.plot(goals[:, 0], goals[:, 1], marker='o', linestyle='-', label="Odom Path", color='blue')

    # Plot obstacles
    plt.scatter(right_obstacles[:, 0], right_obstacles[:, 1], color='red', marker='x', label="Obstacles", s=100)
    plt.scatter(left_obstacles[:, 0], left_obstacles[:, 1], color='red', marker='x', label="Obstacles", s=100)

    # Invert Y-axis if needed
    # plt.gca().invert_yaxis() 
    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    # Save
    plt.savefig("Plots/BasicObstacles.png", bbox_inches='tight', pad_inches=0)
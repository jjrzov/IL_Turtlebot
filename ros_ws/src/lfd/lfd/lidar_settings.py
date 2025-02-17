import math

# World Settings
RESOLUTION = 0.1 # Gazebo defined world was 100 x 100, thus 1000 x 1000 grid
X_MAP_ORIGIN = 0
Y_MAP_ORIGIN = 0
X_MAP_MIN = -50
Y_MAP_MIN = -50 # Y grows downward

# LIDAR Metadata (in radians)
SAMPLES = 640
LIDAR_RES = 1
NUM_RAYS = SAMPLES * LIDAR_RES

FOV = 2 * (math.pi / 2.25)  # 160 degrees
HALF_FOV = FOV / 2
MIN_ANGLE = - FOV / 2
MAX_ANGLE = FOV / 2
DELTA_ANGLE = FOV / NUM_RAYS

MAX_DEPTH = 2
MIN_DEPTH = 0.08
LIN_RES = 0.01
import rclpy
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rclpy.serialization
import pandas as pd

def extract_cmd_vel_to_csv(bag_path, output_folder):
    rclpy.init()

    # Set up the reader
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Prepare data storage
    cmd_vel_data = []
    scan_data = []
    odometry_data = []

    # Read messages from the bag
    while reader.has_next():
        topic, msg, timestamp = reader.read_next()
        if topic == "/diff_drive/cmd_vel":  # Process /diff_drive/cmd_vel
            try:
                twist = rclpy.serialization.deserialize_message(msg, Twist)
                cmd_vel_data.append({
                    "timestamp": timestamp,
                    "linear_x": twist.linear.x,
                    "linear_y": twist.linear.y,
                    "linear_z": twist.linear.z,
                    "angular_x": twist.angular.x,
                    "angular_y": twist.angular.y,
                    "angular_z": twist.angular.z,
                })
            except Exception as e:
                print(f"Error deserializing message on /diff_drive/cmd_vel: {e}")

        elif topic == "/diff_drive/scan":   # Process /diff_drive/scan (LIDAR data)
            try:
                scan = rclpy.serialization.deserialize_message(msg, LaserScan)
                scan_data.append({
                    "timestamp": timestamp,
                    "angle_min": scan.angle_min,
                    "angle_max": scan.angle_max,
                    "angle_increment": scan.angle_increment,
                    "time_increment": scan.time_increment,
                    "range_min": scan.range_min,
                    "range_max": scan.range_max,
                    "ranges": ",".join(map(str, scan.ranges)),  # Save ranges as a comma-separated string
                    # "intensities": ",".join(map(str, scan.intensities)) if scan.intensities else "",
                })
            except Exception as e:
                print(f"Error deserializing message on /diff_drive/scan: {e}")

        elif topic == "/diff_drive/odometry":   # Process /diff_drive/scan (LIDAR data)
            try:
                odom = rclpy.serialization.deserialize_message(msg, Odometry)
                odometry_data.append({
                    "timestamp": timestamp,
                    "pose_x": odom.pose.pose.position.x,
                    "pose_y": odom.pose.pose.position.y,
                    "pose_z": odom.pose.pose.position.z,
                    "orientation_x": odom.pose.pose.orientation.x,
                    "orientation_y": odom.pose.pose.orientation.y,
                    "orientation_z": odom.pose.pose.orientation.z,
                    "orientation_w": odom.pose.pose.orientation.w,
                    "linear_velocity_x": odom.twist.twist.linear.x,
                    "linear_velocity_y": odom.twist.twist.linear.y,
                    "linear_velocity_z": odom.twist.twist.linear.z,
                    "angular_velocity_x": odom.twist.twist.angular.x,
                    "angular_velocity_y": odom.twist.twist.angular.y,
                    "angular_velocity_z": odom.twist.twist.angular.z,
                })
            except Exception as e:
                print(f"Error deserializing message on /diff_drive/odometry: {e}")


    # Save to CSV if data exists
    if cmd_vel_data:
        cmd_vel_df = pd.DataFrame(cmd_vel_data)
        cmd_vel_csv_file = f"{output_folder}/diff_drive_cmd_vel.csv"
        cmd_vel_df.to_csv(cmd_vel_csv_file, index=False)
        print(f"Saved {len(cmd_vel_data)} messages from /diff_drive/cmd_vel to {cmd_vel_csv_file}")
    else:
        print("No messages found for /diff_drive/cmd_vel.")

    if scan_data:
        scan_df = pd.DataFrame(scan_data)
        scan_csv_file = f"{output_folder}/diff_drive_scan.csv"
        scan_df.to_csv(scan_csv_file, index=False)
        print(f"Saved {len(scan_data)} messages from /diff_drive/scan to {scan_csv_file}")
    else:
        print("No messages found for /diff_drive/scan.")

    if odometry_data:
        odom_df = pd.DataFrame(odometry_data)
        odom_csv_file = f"{output_folder}/diff_drive_odometry.csv"
        odom_df.to_csv(odom_csv_file, index=False)
        print(f"Saved {len(odometry_data)} messages from /diff_drive/odometry to {odom_csv_file}")
    else:
        print("No messages found for /diff_drive/odometry.")

    rclpy.shutdown()

# Example usage
if __name__ == "__main__":
    bag_file_path = "basic_path"  # Path to your ROS2 bag file
    output_folder = "basic_path_csv"  # Desired CSV file path
    extract_cmd_vel_to_csv(bag_file_path, output_folder)
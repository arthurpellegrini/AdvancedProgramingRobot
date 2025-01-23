import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time

def load_json_file(filepath):
    with open(filepath, 'r') as file:
        try:
            return json.load(file)
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON from file {filepath}: {e}")
            return None

def extract_laser_scan_points(laser_scan_data):
    points = []
    angle_increment = laser_scan_data.get("angle_increment", 0)
    ranges = laser_scan_data.get("ranges", [])
    for i, range_val in enumerate(ranges):
        angle = i * angle_increment
        x = range_val * np.cos(angle)
        y = range_val * np.sin(angle)
        points.append((x, y))
    return points

def extract_odometry_pose(odometry_data):
    pose = odometry_data.get("pose", {}).get("pose", {}).get("position", {})
    x = pose.get("x", 0)
    y = pose.get("y", 0)
    theta = odometry_data.get("pose", {}).get("pose", {}).get("orientation", {}).get("z", 0)  # Assuming theta is stored in z
    return [(x, y, theta)]

def visualize_data():
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots(figsize=(10, 5))

    laser_scan_filepath = './tmp/laser_data.json'
    odometry_filepath = './tmp/odometry_data.json'

    while True:
        plt.cla()  # Clear previous frame

        laser_scan_data = load_json_file(laser_scan_filepath)
        odometry_data = load_json_file(odometry_filepath)
    
        if laser_scan_data and odometry_data:
            laser_scan_points = extract_laser_scan_points(laser_scan_data)
            odometry_pose = extract_odometry_pose(odometry_data)

            if laser_scan_points:
                laser_scan_points_df = pd.DataFrame(laser_scan_points, columns=['x', 'y'])
                ax.scatter(laser_scan_points_df['x'], laser_scan_points_df['y'], s=1, label='LaserScan Points')

            if odometry_pose:
                odometry_pose_df = pd.DataFrame(odometry_pose, columns=['x', 'y', 'theta'])
                ax.scatter(odometry_pose_df['x'], odometry_pose_df['y'], color='red', label='Odometry Pose', marker='x')

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('LaserScan and Odometry Data')
            ax.legend()

        plt.pause(1)  # Pause for 1 second before next update

if __name__ == "__main__":
    visualize_data()

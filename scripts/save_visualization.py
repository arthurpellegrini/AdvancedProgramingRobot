import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from datetime import datetime

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

def visualize_data(laser_scan_points, odometry_pose, output_filepath):
    plt.figure(figsize=(10, 5))

    # Plot LaserScan points
    if laser_scan_points:
        laser_scan_points = pd.DataFrame(laser_scan_points, columns=['x', 'y'])
        plt.scatter(laser_scan_points['x'], laser_scan_points['y'], s=1, label='LaserScan Points')

    # Plot Odometry pose
    if odometry_pose:
        odometry_pose = pd.DataFrame(odometry_pose, columns=['x', 'y', 'theta'])
        plt.scatter(odometry_pose['x'], odometry_pose['y'], color='red', label='Odometry Pose', marker='x')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LaserScan and Odometry Data')
    plt.legend()

    # Save visualization to an image file
    plt.savefig(output_filepath)
    plt.close()

if __name__ == "__main__":
    laser_scan_filepath = './tmp/laser_data.json'
    odometry_filepath = './tmp/odometry_data.json'
    
    # Get current timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d-%H%M%S")
    output_filepath = f'./tmp/{timestamp}_Visualization.png'

    laser_scan_data = load_json_file(laser_scan_filepath)
    odometry_data = load_json_file(odometry_filepath)
    
    if laser_scan_data and odometry_data:
        laser_scan_points = extract_laser_scan_points(laser_scan_data)
        odometry_pose = extract_odometry_pose(odometry_data)
        visualize_data(laser_scan_points, odometry_pose, output_filepath)
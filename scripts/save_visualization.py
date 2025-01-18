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
    for scan in laser_scan_data:
        angle_increment = scan.get("angle_increment", 0)
        ranges = scan.get("ranges", [])
        for i, range_val in enumerate(ranges):
            angle = i * angle_increment
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            points.append((x, y))
    return points

def extract_odometry_poses(odometry_data):
    poses = []
    for entry in odometry_data:
        pose = entry.get("pose", {}).get("pose", {}).get("position", {})
        x = pose.get("x", 0)
        y = pose.get("y", 0)
        theta = entry.get("pose", {}).get("pose", {}).get("orientation", {}).get("z", 0) # Assuming theta is stored in z
        poses.append((x, y, theta))
    return poses

def visualize_data(laser_scan_points, odometry_poses, output_filepath):
    plt.figure(figsize=(10, 5))

    # Plot LaserScan points
    if laser_scan_points:
        laser_scan_points = pd.DataFrame(laser_scan_points, columns=['x', 'y'])
        plt.scatter(laser_scan_points['x'], laser_scan_points['y'], s=1, label='LaserScan Points')

    # Plot Odometry poses
    if odometry_poses:
        odometry_poses = pd.DataFrame(odometry_poses, columns=['x', 'y', 'theta'])
        plt.plot(odometry_poses['x'], odometry_poses['y'], label='Odometry Path')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LaserScan and Odometry Data')
    plt.legend()

    # Speichern der Visualisierung in eine Bilddatei
    plt.savefig(output_filepath)
    plt.close()

if __name__ == "__main__":
    laser_scan_filepath = '../tmp/laser_data.json'
    odometry_filepath = '../tmp/odometry_data.json'

    # Ermitteln des aktuellen Zeitstempels
    timestamp = datetime.now().strftime("%Y-%m-%d-%H%M")
    output_filepath = f'../tmp/{timestamp}_Visualization.png'

    laser_scan_data = load_json_file(laser_scan_filepath)
    odometry_data = load_json_file(odometry_filepath)

    if laser_scan_data and odometry_data:
        laser_scan_points = extract_laser_scan_points(laser_scan_data)
        odometry_poses = extract_odometry_poses(odometry_data)

        visualize_data(laser_scan_points, odometry_poses, output_filepath)
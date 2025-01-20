import json
import numpy as np
import matplotlib.pyplot as plt

# Define the file path for LaserScan data
LASERSCAN_FILE = "data_test/test_9997.txt"

# Robot odometry data (global coordinates and orientation)
robot_x = 5.0  # Example X position
robot_y = 5.0  # Example Y position
robot_theta = np.radians(45)  # Example orientation in radians

def clean_line(line):
    """Remove non-JSON content (e.g., timestamps) from a line."""
    line = line.strip()
    if not line:
        return ""  # Skip empty lines
    if line.startswith("[") and line.endswith("]"):
        return ""  # Ignore lines enclosed in brackets
    json_start = line.find("{")
    if json_start != -1:
        return line[json_start:]
    return ""

def parse_laserscan(file_path):
    """Parse LaserScan data from the specified file, cleaning non-JSON lines."""
    extracted_data = []
    with open(file_path, 'r') as f:
        buffer = ""
        inside_json = False

        for line in f:
            line = line.strip()
            if "---START---" in line:
                # Start capturing potential JSON
                inside_json = True
                buffer = ""
            elif "___END___" in line and inside_json:
                # Stop capturing JSON, attempt to parse
                inside_json = False
                try:
                    json_data = json.loads(buffer)
                    # Validate essential fields in the JSON
                    if "ranges" in json_data and "angle_min" in json_data and "angle_increment" in json_data:
                        extracted_data.append(json_data)
                    else:
                        print(f"Invalid LaserScan structure: {buffer[:100]}...")
                except json.JSONDecodeError as e:
                    print(f"Error parsing JSON block: {e}\nBlock content: {buffer[:100]}...")
            elif inside_json:
                # Clean line of non-JSON content and append to the buffer
                cleaned_line = clean_line(line)
                buffer += cleaned_line

    return extracted_data

def process_laserscan(scan_data):
    """Process LaserScan data to extract global obstacle coordinates."""
    angle_min = scan_data["angle_min"]
    angle_increment = scan_data["angle_increment"]
    ranges = scan_data["ranges"]
    range_min = scan_data.get("range_min", 0.1)
    range_max = scan_data.get("range_max", 10.0)

    points_x = []
    points_y = []

    # Process each range
    for i, r in enumerate(ranges):
        if range_min <= r <= range_max:  # Valid range
            angle = angle_min + i * angle_increment  # Calculate angle for the range
            # Convert to local Cartesian coordinates
            local_x = r * np.cos(angle)
            local_y = r * np.sin(angle)

            # Transform to global coordinates
            global_x = robot_x + local_x * np.cos(robot_theta) - local_y * np.sin(robot_theta)
            global_y = robot_y + local_x * np.sin(robot_theta) + local_y * np.cos(robot_theta)

            points_x.append(global_x)
            points_y.append(global_y)

    return points_x, points_y

def plot_map(points_x, points_y, robot_x, robot_y):
    """Plot the processed LaserScan data as a 2D map."""
    plt.figure(figsize=(10, 10))
    plt.scatter(points_x, points_y, label="Obstacles", color="red", s=10)
    plt.scatter(robot_x, robot_y, label="Robot Position", color="blue", s=100)
    plt.title("LaserScan Mapping")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    # Parse LaserScan data from the file
    laser_scans = parse_laserscan(LASERSCAN_FILE)
    if laser_scans:
        # Process the first valid LaserScan data block
        for scan_data in laser_scans:
            points_x, points_y = process_laserscan(scan_data)
            # Plot the map
            if points_x and points_y:
                plot_map(points_x, points_y, robot_x, robot_y)
            else:
                print("No valid points to plot.")
    else:
        print("No valid LaserScan data found in the file.")
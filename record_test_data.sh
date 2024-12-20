#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <ip address> <name of the test>"
    exit 1
fi

# Set the server IP and ports
SERVER_IP=$1
PORT_LASER_SCAN=9997
PORT_ODOM=9998
PORT_CMD_VEL=9999

# Assign the argument to a variable
BASE_NAME=$2

# Create the data_test directory if it doesn't exist
mkdir -p data_test

# Define output file names
OUTPUT_FILE_LASER_SCAN="data_test/test_9997_${BASE_NAME}.txt"
OUTPUT_FILE_ODOM="data_test/test_9998_${BASE_NAME}.txt"

# # Function to clean up background processes on script termination
# cleanup() {
#     echo "Stopping background processes..."
#     kill 0  # Sends SIGTERM to all processes in the current process group
#     exit 0
# }

# # Trap signals to ensure cleanup is called on script termination
# trap cleanup SIGINT SIGTERM EXIT

# Launch the first command in the background
echo "Starting ListenOnTCPPort for port $PORT_ODOM, logging to data_test/$OUTPUT_FILE_ODOM"
sudo ./build/ListenOnTCPPort $SERVER_IP $PORT_ODOM >> data_test/$OUTPUT_FILE_ODOM &

# Launch the second command in the background
echo "Starting ListenOnTCPPort for port $PORT_LASER_SCAN, logging to data_test/$OUTPUT_FILE_LASER_SCAN"
sudo ./build/ListenOnTCPPort $SERVER_IP $PORT_LASER_SCAN >> data_test/$OUTPUT_FILE_LASER_SCAN &

# Wait briefly to ensure background processes start
sleep 1

# Launch the third command interactively
echo "Starting TalkOnTCPPort interactively for port $PORT_CMD_VEL"
./build/TalkOnTCPPort $SERVER_IP $PORT_CMD_VEL


# Add loop for checking the robot is available (ping)
# Add commander
# Implement Shared Memory


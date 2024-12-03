// Implementation File: ros_tcp_listener.cpp

#include "ros_tcp_listener.h"
#include <iostream>
#include <cstring>

void handlePointCloud2Data(const char* data, size_t length) {
    sensor_msgs::PointCloud2 pointCloud;
    // Deserialize the data into ROS message format (this is a placeholder, adjust accordingly)
    // Assume data is already serialized in ROS message format.
    ros::serialization::IStream stream(reinterpret_cast<uint8_t*>(const_cast<char*>(data)), length);
    ros::serialization::deserialize(stream, pointCloud);
    
    // Process the point cloud data
    ROS_INFO("Received PointCloud2 data with width: %u, height: %u", pointCloud.width, pointCloud.height);
}

void handleLaserScanData(const char* data, size_t length) {
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string errs;
    std::istringstream sstream(std::string(data, length));

    if (!Json::parseFromStream(readerBuilder, sstream, &root, &errs)) {
        ROS_ERROR("Failed to parse LaserScan data: %s", errs.c_str());
        return;
    }

    sensor_msgs::LaserScan laserScan;
    try {
        laserScan.header.seq = root["header"]["seq"].asUInt();
        laserScan.header.stamp.sec = root["header"]["stamp"]["secs"].asInt();
        laserScan.header.stamp.nsec = root["header"]["stamp"]["nsecs"].asInt();
        laserScan.header.frame_id = root["header"]["frame_id"].asString();
        laserScan.angle_min = root["angle_min"].asFloat();
        laserScan.angle_max = root["angle_max"].asFloat();
        laserScan.angle_increment = root["angle_increment"].asFloat();
        laserScan.time_increment = root["time_increment"].asFloat();
        laserScan.scan_time = root["scan_time"].asFloat();
        laserScan.range_min = root["range_min"].asFloat();
        laserScan.range_max = root["range_max"].asFloat();

        const Json::Value ranges = root["ranges"];
        for (const auto& range : ranges) {
            laserScan.ranges.push_back(range.asFloat());
        }

        const Json::Value intensities = root["intensities"];
        for (const auto& intensity : intensities) {
            laserScan.intensities.push_back(intensity.asFloat());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while parsing LaserScan data: %s", e.what());
        return;
    }
    
    // Process the laser scan data
    ROS_INFO("Received LaserScan data with range size: %lu", laserScan.ranges.size());
}

void startTCPListener() {
    int serverSock, clientSock;
    struct sockaddr_in serverAddr, clientAddr;
    socklen_t clientLen;
    char buffer[BUFFER_SIZE];

    // Create socket
    if ((serverSock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket() failed");
        return;
    }

    // Set up server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(PORT);

    // Bind the socket to the port
    if (bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind() failed");
        close(serverSock);
        return;
    }

    // Listen for incoming connections
    if (listen(serverSock, 5) < 0) {
        perror("listen() failed");
        close(serverSock);
        return;
    }

    ROS_INFO("Listening for incoming connections on port %d", PORT);

    while (ros::ok()) {
        clientLen = sizeof(clientAddr);
        if ((clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &clientLen)) < 0) {
            perror("accept() failed");
            continue;
        }

        ROS_INFO("Client connected: %s", inet_ntoa(clientAddr.sin_addr));

        ssize_t bytesReceived;
        while ((bytesReceived = recv(clientSock, buffer, BUFFER_SIZE, 0)) > 0) {
            // Assuming first byte indicates the message type: 1 for PointCloud2, 2 for LaserScan
            if (buffer[0] == 1) {
                handlePointCloud2Data(buffer + 1, bytesReceived - 1);
            } else if (buffer[0] == 2) {
                handleLaserScanData(buffer + 1, bytesReceived - 1);
            } else {
                ROS_WARN("Unknown message type received");
            }
        }

        if (bytesReceived < 0) {
            perror("recv() failed");
        } else {
            ROS_INFO("Client disconnected");
        }

        close(clientSock);
    }

    close(serverSock);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_listener_node");
    ros::NodeHandle nh;
    startTCPListener();
    return 0;
}

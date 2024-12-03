#include "RosTCPLaserListener.h"
#include "HandlePointCloud2Data.h"
#include "HandleLaserScanData.h"
#include "CreateTCPServerSocket.h"
#include "ConnectToServer.h"
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

volatile bool keepRunning = true;

void intHandler(int dummy) {
    keepRunning = false;
    ROS_INFO("Shutting down TCP Listener...");
}

void setSocketNonBlocking(int socket) {
    int flags = fcntl(socket, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl(F_GETFL) failed");
        return;
    }
    if (fcntl(socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl(F_SETFL) failed");
    }
}

void startTCPListener(unsigned short port, ros::Publisher& laserScanPub) {
    int serverSock, clientSock;
    struct sockaddr_in clientAddr;
    socklen_t clientLen;
    char buffer[BUFFER_SIZE];

    serverSock = CreateTCPServerSocket(port);
    if (serverSock < 0) {
        ROS_ERROR("Failed to create server socket on port %d", port);
        return;
    }
    ROS_INFO("Server socket created successfully. Listening on port %d", port);

    setSocketNonBlocking(serverSock);

    while (ros::ok() && keepRunning) {
        clientLen = sizeof(clientAddr);
        clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientSock < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                usleep(100000); // Sleep for 100ms to avoid busy-waiting
                continue;
            } else if (!keepRunning) {
                break;
            }
            perror("accept() failed");
            continue;
        }

        ROS_INFO("Client connected: %s", inet_ntoa(clientAddr.sin_addr));

        setSocketNonBlocking(clientSock);

        ssize_t bytesReceived;
        while (keepRunning) {
            bytesReceived = recv(clientSock, buffer, BUFFER_SIZE, 0);
            if (bytesReceived < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    usleep(100000); // Sleep for 100ms to avoid busy-waiting
                    continue;
                } else {
                    perror("recv() failed");
                    break;
                }
            } else if (bytesReceived == 0) {
                ROS_INFO("Client disconnected");
                break;
            }

            // Assuming first byte indicates the message type: 1 for PointCloud2, 2 for LaserScan
            if (buffer[0] == 1) {
                handlePointCloud2Data(buffer + 1, bytesReceived - 1);
            } else if (buffer[0] == 2) {
                sensor_msgs::LaserScan scanMsg;
                // Hier sollte der empfangene Puffer in die LaserScan-Nachricht konvertiert werden
                // Zum Beispiel:
                // deserializeLaserScan(buffer + 1, bytesReceived - 1, scanMsg);
                laserScanPub.publish(scanMsg);
                ROS_INFO("Published LaserScan data");
            } else {
                ROS_WARN("Unknown message type received");
            }
        }

        close(clientSock);
    }

    close(serverSock);
    ROS_INFO("Server socket closed");
}

int main(int argc, char** argv) {
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);

    ros::init(argc, argv, "tcp_listener_node");
    ros::NodeHandle nh;

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <Port>" << std::endl;
        return 1;
    }

    unsigned short port = static_cast<unsigned short>(std::stoi(argv[1]));

    ros::Publisher laserScanPub = nh.advertise<sensor_msgs::LaserScan>("/tcp_laser_scan", 10);

    startTCPListener(port, laserScanPub);
    return 0;
}

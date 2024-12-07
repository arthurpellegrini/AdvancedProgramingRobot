#include "RosTCPOdometryListener.h"
#include "HandleOdometryData.h"
#include "CreateTCPServerSocket.h"
#include "ConnectToServer.h"

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

void startTCPListener(const char* serverIP, unsigned short port, ros::Publisher& odometryPub) {
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

            // Assuming first byte indicates the message type: 1 for PointCloud2, 2 for LaserScan, 3 for Odometry
            if (buffer[0] == 3) {
                nav_msgs::Odometry odometryMsg;
                handleOdometryData(buffer + 1, bytesReceived - 1);
                odometryPub.publish(odometryMsg);
                ROS_INFO("Published Odometry data");
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

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <Server IP> <Port>" << std::endl;
        return 1;
    }

    const char* serverIP = argv[1];
    unsigned short port = static_cast<unsigned short>(std::stoi(argv[2]));

    ros::Publisher odometryPub = nh.advertise<nav_msgs::Odometry>("/tcp_odometry_data", 10);

    startTCPListener(serverIP, port, odometryPub);
    return 0;
}

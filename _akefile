# Compiler
CC = g++

CFLAGS = -Wall -Iinclude -I/opt/ros/noetic/include -pthread
LDFLAGS = -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -ljsoncpp -pthread

# C-Compiler für .c-Dateien
C_COMPILER = gcc

# Target executable
PROGRAMS  = tcp_server tcp_client tcp_crawler ChattyTCPServer ListenOnTCPPort TalkOnTCPPort Ros_LaserScan_Listener Ros_TCP_Odometry_Listener


# Source and object files
tcp_server_SRCS = src/TCPEchoServer.c src/CreateTCPServerSocket.c src/AcceptTCPConnection.c src/HandleTCPClient.c src/DieWithError.c
tcp_server_OBJS = src/TCPEchoServer.o src/CreateTCPServerSocket.o src/AcceptTCPConnection.o src/HandleTCPClient.o src/DieWithError.o

tcp_client_SRCS = src/TCPEchoClient.c src/DieWithError.c
tcp_client_OBJS = src/TCPEchoClient.o src/DieWithError.o

tcp_crawler_SRCS = src/main.cpp src/ConnectToServer.cpp src/LaserScanHandler.cpp src/OdometryHandler.cpp
tcp_crawler_OBJS = src/main.o src/ConnectToServer.o src/LaserScanHandler.o src/OdometryHandler.o

ChattyTCPServer_SRCS = src/ChattyTCPServer.cpp src/SendSimulatedData.cpp
ChattyTCPServer_OBJS = src/ChattyTCPServer.o src/SendSimulatedData.o

ListenOnTCPPort_SRCS = src/ListenOnTCPPort.cpp src/ConnectToServer.cpp
ListenOnTCPPort_OBJS = src/ListenOnTCPPort.o src/ConnectToServer.o

TalkOnTCPPort_SRCS = src/TalkOnTCPPort.cpp src/ConnectToServer.cpp
TalkOnTCPPort_OBJS = src/TalkOnTCPPort.o src/ConnectToServer.o

Ros_LaserScan_Listener_SRCS = src/RosLocalHostLaserListener.cpp src/HandleLaserScanData.cpp src/HandlePointCloud2Data.cpp src/CreateTCPServerSocket.c src/DieWithError.c src/ConnectToServer.cpp
Ros_LaserScan_Listener_OBJS = src/RosLocalHostLaserListener.o src/HandleLaserScanData.o src/HandlePointCloud2Data.o src/CreateTCPServerSocket.o src/DieWithError.o src/ConnectToServer.o

Ros_TCP_LaserScan_Listener_SRCS = src/RosTCPLaserListener.cpp src/HandleLaserScanData.cpp src/HandlePointCloud2Data.cpp src/CreateTCPServerSocket.c src/DieWithError.c src/ConnectToServer.cpp
Ros_TCP_LaserScan_Listener_OBJS = src/RosTCPLaserListener.o src/HandleLaserScanData.o src/HandlePointCloud2Data.o src/CreateTCPServerSocket.o src/DieWithError.o src/ConnectToServer.o

Ros_TCP_Odometry_Listener_SRCS = src/RosTCPOdometryListener.cpp src/HandleOdometryData.cpp src/CreateTCPServerSocket.c src/DieWithError.c src/ConnectToServer.cpp
Ros_TCP_Odometry_Listener_OBJS = src/RosTCPOdometryListener.o src/HandleOdometryData.o src/CreateTCPServerSocket.o src/DieWithError.o src/ConnectToServer.o


# Default rule: Build all programs
all: $(PROGRAMS)

# Rule to build tcp_server
tcp_server: $(tcp_server_OBJS)
	$(CC) $(CFLAGS) -o tcp_server $(tcp_server_OBJS)

# Rule to build tcp_client
tcp_client: $(tcp_client_OBJS)
	$(CC) $(CFLAGS) -o tcp_client $(tcp_client_OBJS)	

# Rule to build tcp_crawler
tcp_crawler: $(tcp_crawler_OBJS)
	$(CC) $(CFLAGS) -o tcp_crawler $(tcp_crawler_OBJS)

# Rule to build ChattyTCPServer
ChattyTCPServer: $(ChattyTCPServer_OBJS)
	$(CC) $(CFLAGS) -o ChattyTCPServer $(ChattyTCPServer_OBJS)

# Rule to build listenOnTCPPort
ListenOnTCPPort: $(ListenOnTCPPort_OBJS)
	$(CC) $(CFLAGS) -o ListenOnTCPPort $(ListenOnTCPPort_OBJS)

# Rule to build talkOnTCPPort
TalkOnTCPPort: $(TalkOnTCPPort_OBJS)
	$(CC) $(CFLAGS) -o TalkOnTCPPort $(TalkOnTCPPort_OBJS)

# Rule to build Ros_LaserScan_Listener
Ros_LaserScan_Listener: $(Ros_LaserScan_Listener_OBJS)
	$(CC) $(CFLAGS) -o Ros_LaserScan_Listener $(Ros_LaserScan_Listener_OBJS) $(LDFLAGS)

# Rule to build Ros_TCP_LaserScan_Listener
Ros_TCP_LaserScan_Listener: $(Ros_TCP_LaserScan_Listener_OBJS)
	$(CC) $(CFLAGS) -o Ros_TCP_LaserScan_Listener $(Ros_TCP_LaserScan_Listener_OBJS) $(LDFLAGS)

# Rule to build Ros_TCP_Odometry_Listener
Ros_TCP_Odometry_Listener: $(Ros_TCP_Odometry_Listener_OBJS)
	$(CC) $(CFLAGS) -o Ros_TCP_Odometry_Listener $(Ros_TCP_Odometry_Listener_OBJS) $(LDFLAGS)


# Compile .c files
src/%.o: src/%.c
	$(C_COMPILER) -Wall -Iinclude -c $< -o $@

# Compile .cpp files
src/%.o: src/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule to remove generated files
clean:
	rm -f $(tcp_server_OBJS) $(tcp_client_OBJS) $(tcp_crawler_OBJS) $(ChattyTCPServer_OBJS) $(ListenOnTCPPort_OBJS) $(TalkOnTCPPort_OBJS) $(Ros_LaserScan_Listener_OBJS) $(Ros_TCP_LaserScan_Listener_OBJS) $(Ros_TCP_Odometry_Listener_OBJS) $(PROGRAMS) 

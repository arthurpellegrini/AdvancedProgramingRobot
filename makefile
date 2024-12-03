# Compiler
CC = g++
#CFLAGS = -Wall -Iinclude -pthread
CFLAGS = -Wall -Iinclude -I/opt/ros/noetic/include -pthread
LDFLAGS = -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -ljsoncpp -pthread

# C-Compiler für .c-Dateien
C_COMPILER = gcc

# Target executable
#PROGRAMS  = tcp_server tcp_client tcp_crawler ChattyTCPServer ListenOnTCPPort TalkOnTCPPort
PROGRAMS  = Ros_LaserScan_Listener

# Source and object files
tcp_server_SRCS = sockets/TCPEchoServer.c sockets/CreateTCPServerSocket.c sockets/AcceptTCPConnection.c sockets/HandleTCPClient.c sockets/DieWithError.c
tcp_server_OBJS = sockets/TCPEchoServer.o sockets/CreateTCPServerSocket.o sockets/AcceptTCPConnection.o sockets/HandleTCPClient.o sockets/DieWithError.o

tcp_client_SRCS = sockets/TCPEchoClient.c sockets/DieWithError.c
tcp_client_OBJS = sockets/TCPEchoClient.o sockets/DieWithError.o

tcp_crawler_SRCS = sockets/main.cpp sockets/ConnectToServer.cpp sockets/LaserScanHandler.cpp sockets/OdometryHandler.cpp
tcp_crawler_OBJS = sockets/main.o sockets/ConnectToServer.o sockets/LaserScanHandler.o sockets/OdometryHandler.o

ChattyTCPServer_SRCS = sockets/ChattyTCPServer.cpp sockets/SendSimulatedData.cpp
ChattyTCPServer_OBJS = sockets/ChattyTCPServer.o sockets/SendSimulatedData.o

ListenOnTCPPort_SRCS = sockets/ListenOnTCPPort.cpp sockets/ConnectToServer.cpp
ListenOnTCPPort_OBJS = sockets/ListenOnTCPPort.o sockets/ConnectToServer.o

TalkOnTCPPort_SRCS = sockets/TalkOnTCPPort.cpp sockets/ConnectToServer.cpp
TalkOnTCPPort_OBJS = sockets/TalkOnTCPPort.o sockets/ConnectToServer.o

Ros_LaserScan_Listener_SRCS = sockets/RosTCPLaserListener.cpp sockets/HandleLaserScanData.cpp sockets/HandlePointCloud2Data.cpp sockets/CreateTCPServerSocket.c sockets/DieWithError.c sockets/ConnectToServer.cpp
Ros_LaserScan_Listener_OBJS = sockets/RosTCPLaserListener.o sockets/HandleLaserScanData.o sockets/HandlePointCloud2Data.o sockets/CreateTCPServerSocket.o sockets/DieWithError.o sockets/ConnectToServer.o

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

# Compile .c files
sockets/%.o: sockets/%.c
	$(C_COMPILER) -Wall -Iinclude -c $< -o $@

# Compile .cpp files
sockets/%.o: sockets/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule to remove generated files
clean:
	rm -f $(tcp_server_OBJS) $(tcp_client_OBJS) $(tcp_crawler_OBJS) $(ChattyTCPServer_OBJS) $(ListenOnTCPPort_OBJS) $(TalkOnTCPPort_OBJS) $(tcp_listener_node_OBJS) $(PROGRAMS) 

# Compiler
CC = g++
CFLAGS = -Wall -Iinclude -I/opt/ros/noetic/include -pthread
LDFLAGS = -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -ljsoncpp -pthread

# C-Compiler pour les fichiers .c
C_COMPILER = gcc

# Répertoires
BUILD_DIR = build
SRC_DIR = src

# Target executables
PROGRAMS = tcp_server tcp_client tcp_crawler ChattyTCPServer ListenOnTCPPort TalkOnTCPPort Ros_LaserScan_Listener Ros_TCP_LaserScan_Listener Ros_TCP_Odometry_Listener
PROGRAM_PATHS = $(addprefix $(BUILD_DIR)/, $(PROGRAMS))

# Source and object files
tcp_server_SRCS = $(SRC_DIR)/TCPEchoServer.c $(SRC_DIR)/CreateTCPServerSocket.c $(SRC_DIR)/AcceptTCPConnection.c $(SRC_DIR)/HandleTCPClient.c $(SRC_DIR)/DieWithError.c
tcp_server_OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(tcp_server_SRCS))

tcp_client_SRCS = $(SRC_DIR)/TCPEchoClient.c $(SRC_DIR)/DieWithError.c
tcp_client_OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(tcp_client_SRCS))

tcp_crawler_SRCS = $(SRC_DIR)/main.cpp $(SRC_DIR)/ConnectToServer.cpp $(SRC_DIR)/LaserScanHandler.cpp $(SRC_DIR)/OdometryHandler.cpp
tcp_crawler_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(tcp_crawler_SRCS))

ChattyTCPServer_SRCS = $(SRC_DIR)/ChattyTCPServer.cpp $(SRC_DIR)/SendSimulatedData.cpp
ChattyTCPServer_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(ChattyTCPServer_SRCS))

ListenOnTCPPort_SRCS = $(SRC_DIR)/ListenOnTCPPort.cpp $(SRC_DIR)/ConnectToServer.cpp
ListenOnTCPPort_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(ListenOnTCPPort_SRCS))

TalkOnTCPPort_SRCS = $(SRC_DIR)/TalkOnTCPPort.cpp $(SRC_DIR)/ConnectToServer.cpp
TalkOnTCPPort_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(TalkOnTCPPort_SRCS))

Ros_LaserScan_Listener_SRCS = $(SRC_DIR)/RosLocalHostLaserListener.cpp $(SRC_DIR)/HandleLaserScanData.cpp $(SRC_DIR)/HandlePointCloud2Data.cpp $(SRC_DIR)/CreateTCPServerSocket.c $(SRC_DIR)/DieWithError.c $(SRC_DIR)/ConnectToServer.cpp
Ros_LaserScan_Listener_OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(Ros_LaserScan_Listener_SRCS)))

Ros_TCP_LaserScan_Listener_SRCS = $(SRC_DIR)/RosTCPLaserListener.cpp $(SRC_DIR)/HandleLaserScanData.cpp $(SRC_DIR)/HandlePointCloud2Data.cpp $(SRC_DIR)/CreateTCPServerSocket.c $(SRC_DIR)/DieWithError.c $(SRC_DIR)/ConnectToServer.cpp
Ros_TCP_LaserScan_Listener_OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(Ros_TCP_LaserScan_Listener_SRCS)))

Ros_TCP_Odometry_Listener_SRCS = $(SRC_DIR)/RosTCPOdometryListener.cpp $(SRC_DIR)/HandleOdometryData.cpp $(SRC_DIR)/CreateTCPServerSocket.c $(SRC_DIR)/DieWithError.c $(SRC_DIR)/ConnectToServer.cpp
Ros_TCP_Odometry_Listener_OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(Ros_TCP_Odometry_Listener_SRCS)))

# Règle principale
all: $(BUILD_DIR) $(PROGRAM_PATHS)

# Créer les exécutables
$(BUILD_DIR)/tcp_server: $(tcp_server_OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/tcp_client: $(tcp_client_OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/tcp_crawler: $(tcp_crawler_OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/ChattyTCPServer: $(ChattyTCPServer_OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/ListenOnTCPPort: $(ListenOnTCPPort_OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/TalkOnTCPPort: $(TalkOnTCPPort_OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/Ros_LaserScan_Listener: $(Ros_LaserScan_Listener_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/Ros_TCP_LaserScan_Listener: $(Ros_TCP_LaserScan_Listener_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/Ros_TCP_Odometry_Listener: $(Ros_TCP_Odometry_Listener_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile les fichiers .c
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(C_COMPILER) -Wall -Iinclude -c $< -o $@

# Compile les fichiers .cpp
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Créer le dossier build s'il n'existe pas
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Nettoyage
clean:
	rm -rf $(BUILD_DIR)

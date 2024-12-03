# Compiler
CC = g++
CFLAGS = -Wall -Iinclude

# C-Compiler für .c-Dateien
C_COMPILER = gcc

# Target executable
PROGRAMS  = tcp_server tcp_client tcp_crawler ChattyTCPServer ListenOnTCPPort

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


# Compile .c files
sockets/%.o: sockets/%.c
	$(C_COMPILER) -Wall -Iinclude -c $< -o $@

# Compile .cpp files
sockets/%.o: sockets/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule to remove generated files
clean:
	rm -f $(tcp_server_OBJS) $(tcp_client_OBJS) $(tcp_crawler_OBJS) $(ChattyTCPServer_OBJS) $(listenOnTCPPort_OBJS) $(PROGRAMS)

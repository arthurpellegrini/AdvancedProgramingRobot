# Compiler
CC = g++
CFLAGS = -Wall -Iinclude

# C-Compiler für .c-Dateien
C_COMPILER = gcc

# Target executable
PROGRAMS  = tcp_server tcp_client

# Source and object files
tcp_server_SRCS = sockets/main.cpp sockets/CreateTCPServerSocket.c sockets/AcceptTCPConnection.c sockets/HandleTCPClient.c sockets/DieWithError.c
tcp_server_OBJS = sockets/main.o sockets/CreateTCPServerSocket.o sockets/AcceptTCPConnection.o sockets/HandleTCPClient.o sockets/DieWithError.o

tcp_client_SRCS = sockets/TCPEchoClient.c sockets/DieWithError.c
tcp_client_OBJS = sockets/TCPEchoClient.o sockets/DieWithError.o

# Default rule: Build all programs
all: $(PROGRAMS)

# Rule to build tcp_server
tcp_server: $(tcp_server_OBJS)
	$(CC) $(CFLAGS) -o tcp_server $(tcp_server_OBJS)

# Rule to build tcp_client
tcp_client: $(tcp_client_OBJS)
	$(CC) $(CFLAGS) -o tcp_client $(tcp_client_OBJS)	

# Compile .c files
sockets/%.o: sockets/%.c
	$(C_COMPILER) -Wall -Iinclude -c $< -o $@

# Compile .cpp files
sockets/%.o: sockets/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule to remove generated files
clean:
	rm -f $(tcp_server_OBJS) $(tcp_client_OBJS) $(PROGRAMS)

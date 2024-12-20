# Compiler
CC = gcc
CXX = g++
CFLAGS = -Wall -Iinclude
LDFLAGS = -lpthread

# Directories
CLIENT_SERVER_SRC_DIR = ./examples/client_server_communication
CLIENT_SERVER_BUILD_DIR = ./build/client_server_communication
THREADS_SRC_DIR = ./examples/processes_threads
THREADS_BUILD_DIR = ./build/processes_threads
SHARED_MEMORY_SRC_DIR = ./examples/shared_memory
SHARED_MEMORY_BUILD_DIR = ./build/shared_memory

# Source files for client_server_communication programs
tcp_client_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoClient.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c
tcp_server_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoServer.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c $(CLIENT_SERVER_SRC_DIR)/HandleTCPClient.c
tcp_server_fork_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoServer-Fork.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c $(CLIENT_SERVER_SRC_DIR)/HandleTCPClient.c $(CLIENT_SERVER_SRC_DIR)/AcceptTCPConnection.c $(CLIENT_SERVER_SRC_DIR)/CreateTCPServerSocket.c
tcp_server_fork_sigchld_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoServer-Fork-SIGCHLD.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c $(CLIENT_SERVER_SRC_DIR)/HandleTCPClient.c $(CLIENT_SERVER_SRC_DIR)/AcceptTCPConnection.c $(CLIENT_SERVER_SRC_DIR)/CreateTCPServerSocket.c
tcp_server_forkn_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoServer-ForkN.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c $(CLIENT_SERVER_SRC_DIR)/HandleTCPClient.c $(CLIENT_SERVER_SRC_DIR)/AcceptTCPConnection.c $(CLIENT_SERVER_SRC_DIR)/CreateTCPServerSocket.c
tcp_server_select_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoServer-Select.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c $(CLIENT_SERVER_SRC_DIR)/HandleTCPClient.c $(CLIENT_SERVER_SRC_DIR)/AcceptTCPConnection.c $(CLIENT_SERVER_SRC_DIR)/CreateTCPServerSocket.c
tcp_server_thread_SRCS = $(CLIENT_SERVER_SRC_DIR)/TCPEchoServer-Thread.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c $(CLIENT_SERVER_SRC_DIR)/HandleTCPClient.c $(CLIENT_SERVER_SRC_DIR)/AcceptTCPConnection.c $(CLIENT_SERVER_SRC_DIR)/CreateTCPServerSocket.c
broadcast_receiver_SRCS = $(CLIENT_SERVER_SRC_DIR)/BroadcastReceiver.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c
broadcast_sender_SRCS = $(CLIENT_SERVER_SRC_DIR)/BroadcastSender.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c
udp_client_SRCS = $(CLIENT_SERVER_SRC_DIR)/UDPEchoClient.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c
udp_server_SRCS = $(CLIENT_SERVER_SRC_DIR)/UDPEchoServer.c $(CLIENT_SERVER_SRC_DIR)/DieWithError.c

# Source files for processes_threads programs
pthread1_SRCS = $(THREADS_SRC_DIR)/pthread1.c
race_demo_SRCS = $(THREADS_SRC_DIR)/race_demo.c
mutex1_SRCS = $(THREADS_SRC_DIR)/mutex1.c
join1_SRCS = $(THREADS_SRC_DIR)/join1.c
sem1_SRCS = $(THREADS_SRC_DIR)/sem1.c
sem2_1_SRCS = $(THREADS_SRC_DIR)/sem2_1.c
sem2_2_SRCS = $(THREADS_SRC_DIR)/sem2_2.c
sem3_SRCS = $(THREADS_SRC_DIR)/sem3.c
semabinit_SRCS = $(THREADS_SRC_DIR)/semabinit.c
sema_SRCS = $(THREADS_SRC_DIR)/sema.c
semb_SRCS = $(THREADS_SRC_DIR)/semb.c

# Source files for shared_memory programs
thrd_posix_SRCS = $(SHARED_MEMORY_SRC_DIR)/thrd-posix.c
fork_exec_SRCS = $(SHARED_MEMORY_SRC_DIR)/fork_exec.c
newproc_posix_SRCS = $(SHARED_MEMORY_SRC_DIR)/newproc-posix.c
shm_prod_cons_SRCS = $(SHARED_MEMORY_SRC_DIR)/shm-prod-cons.cpp
shm_sem_prod_cons_SRCS = $(SHARED_MEMORY_SRC_DIR)/shm-sem-prod-cons.cpp
msgq_prod_cons_SRCS = $(SHARED_MEMORY_SRC_DIR)/msgq-prod-cons.cpp

# Programs
CLIENT_SERVER_PROGRAMS = TCPEchoClient TCPEchoServer TCPEchoServer-Fork TCPEchoServer-Fork-SIGCHLD \
           TCPEchoServer-ForkN TCPEchoServer-Select TCPEchoServer-Thread \
           BroadcastReceiver BroadcastSender UDPEchoClient UDPEchoServer
THREADS_PROGRAMS = pthread1 race_demo mutex1 join1 sem1 sem2_1 sem2_2 sem3 semabinit sema semb
SHARED_MEMORY_PROGRAMS = thrd-posix fork_exec newproc-posix shm-prod-cons shm-sem-prod-cons msgq-prod-cons

# Default target
all: $(CLIENT_SERVER_BUILD_DIR) $(THREADS_BUILD_DIR) $(SHARED_MEMORY_BUILD_DIR) \
     $(addprefix $(CLIENT_SERVER_BUILD_DIR)/, $(CLIENT_SERVER_PROGRAMS)) \
     $(addprefix $(THREADS_BUILD_DIR)/, $(THREADS_PROGRAMS)) \
     $(addprefix $(SHARED_MEMORY_BUILD_DIR)/, $(SHARED_MEMORY_PROGRAMS))

# Build client_server_communication executables
client_server_communication: $(addprefix $(CLIENT_SERVER_BUILD_DIR)/, $(CLIENT_SERVER_PROGRAMS))

# Build processes_threads executables
processes_threads: $(addprefix $(THREADS_BUILD_DIR)/, $(THREADS_PROGRAMS))

# Build shared_memory executables
shared_memory: $(addprefix $(SHARED_MEMORY_BUILD_DIR)/, $(SHARED_MEMORY_PROGRAMS))

# Build client_server_communication executables 
$(CLIENT_SERVER_BUILD_DIR)/TCPEchoClient: $(tcp_client_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/TCPEchoServer: $(tcp_server_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/TCPEchoServer-Fork: $(tcp_server_fork_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/TCPEchoServer-Fork-SIGCHLD: $(tcp_server_fork_sigchld_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/TCPEchoServer-ForkN: $(tcp_server_forkn_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/TCPEchoServer-Select: $(tcp_server_select_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/TCPEchoServer-Thread: $(tcp_server_thread_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(CLIENT_SERVER_BUILD_DIR)/BroadcastReceiver: $(broadcast_receiver_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/BroadcastSender: $(broadcast_sender_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/UDPEchoClient: $(udp_client_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(CLIENT_SERVER_BUILD_DIR)/UDPEchoServer: $(udp_server_SRCS) | $(CLIENT_SERVER_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

# Build processes_threads executables
$(THREADS_BUILD_DIR)/pthread1: $(pthread1_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/race_demo: $(race_demo_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/mutex1: $(mutex1_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/join1: $(join1_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/sem1: $(sem1_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/sem2_1: $(sem2_1_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/sem2_2: $(sem2_2_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/sem3: $(sem3_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/semabinit: $(semabinit_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/sema: $(sema_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(THREADS_BUILD_DIR)/semb: $(semb_SRCS) | $(THREADS_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

# Build shared_memory executables
$(SHARED_MEMORY_BUILD_DIR)/thrd-posix: $(thrd_posix_SRCS) | $(SHARED_MEMORY_BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(SHARED_MEMORY_BUILD_DIR)/fork_exec: $(fork_exec_SRCS) | $(SHARED_MEMORY_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(SHARED_MEMORY_BUILD_DIR)/newproc-posix: $(newproc_posix_SRCS) | $(SHARED_MEMORY_BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(SHARED_MEMORY_BUILD_DIR)/shm-prod-cons: $(shm_prod_cons_SRCS) | $(SHARED_MEMORY_BUILD_DIR)
	$(CXX) $(CFLAGS) $^ -o $@

$(SHARED_MEMORY_BUILD_DIR)/shm-sem-prod-cons: $(shm_sem_prod_cons_SRCS) | $(SHARED_MEMORY_BUILD_DIR)
	$(CXX) $(CFLAGS) $^ -o $@

$(SHARED_MEMORY_BUILD_DIR)/msgq-prod-cons: $(msgq_prod_cons_SRCS) | $(SHARED_MEMORY_BUILD_DIR)
	$(CXX) $(CFLAGS) $^ -o $@

# Create build directories if they don't exist
$(CLIENT_SERVER_BUILD_DIR):
	mkdir -p $@

$(THREADS_BUILD_DIR):
	mkdir -p $@

$(SHARED_MEMORY_BUILD_DIR):
	mkdir -p $@

# Clean up build artifacts
clean:
	rm -rf $(CLIENT_SERVER_BUILD_DIR) $(THREADS_BUILD_DIR) $(SHARED_MEMORY_BUILD_DIR)
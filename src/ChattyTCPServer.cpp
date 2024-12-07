#include "ChattyTCPServer.h"
#include "SendSimulatedData.h"

void StartChattyTCPServer(unsigned short port) {
    int serverSock, clientSock;
    struct sockaddr_in serverAddr, clientAddr;
    unsigned int clientLen;

    // Create the socket
    if ((serverSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        perror("socket() failed");
        exit(1);
    }

    // Set up the server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(port);

    // Bind the socket to the port
    if (bind(serverSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind() failed");
        close(serverSock);
        exit(1);
    }

    // Listen for incoming connections
    if (listen(serverSock, 5) < 0) {
        perror("listen() failed");
        close(serverSock);
        exit(1);
    }
    if (port == 9997) {
        std::cout << "Simulating LaserScan data on port " << port << std::endl;
    } else if (port == 9998) {
        std::cout << "Simulating Odometry data on port " << port << std::endl;
    } else if (port == 9999) {
        std::cout << "Listening for data on port " << port << std::endl;
    } else
        std::cout << "Chatting on port " << port << std::endl;
        
    // Handle clients in a loop
    for (;;) {
        clientLen = sizeof(clientAddr);
        if ((clientSock = accept(serverSock, (struct sockaddr *)&clientAddr, &clientLen)) < 0) {
            perror("accept() failed");
            continue;
        }

        std::cout << "Client connected: " << inet_ntoa(clientAddr.sin_addr) << std::endl;

        if (port == 9997) {
            SendSimulatedData(clientSock, "LaserScan");
        } else if (port == 9998) {
            SendSimulatedData(clientSock, "Odometry");
        } else if (port == 9999) {
            char buffer[1024];
            ssize_t bytesReceived;
            std::cout << "Listening for data on port 9999..." << std::endl;
            
            // Receive data in a loop
            while ((bytesReceived = recv(clientSock, buffer, sizeof(buffer) - 1, 0)) > 0) {
                buffer[bytesReceived] = '\0'; // Null-terminate the received data
                std::cout << "Received: " << buffer << std::endl;
            }

            if (bytesReceived < 0) {
                perror("recv() failed");
            } else {
                std::cout << "Client disconnected." << std::endl;
            }
        }

        close(clientSock);
    }

    close(serverSock);
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <Port>" << std::endl;
        return 1;
    }

    unsigned short port = std::atoi(argv[1]);
    StartChattyTCPServer(port);

    return 0;
}

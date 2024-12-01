#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "CreateTCPServerSocket.h"
#include "AcceptTCPConnection.h"
#include "HandleTCPClient.h"
#include "DieWithError.h"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <Port>" << std::endl;
        exit(1);
    }

    unsigned short port = std::atoi(argv[1]); // Read the port from arguments

    // Create the server socket
    int serverSock = CreateTCPServerSocket(port);
    std::cout << "Server is listening on port " << port << std::endl;

    for (;;)
    {
        // Accept a new connection
        int clientSock = AcceptTCPConnection(serverSock);

        // Handle the client connection
        HandleTCPClient(clientSock);
    }

    close(serverSock); // Close the server socket (this is never reached)
    return 0;
}

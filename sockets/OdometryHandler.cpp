#include "OdometryHandler.h"

// Function to receive and save Odometry data
void ReceiveAndSaveOdometryData(int sock) {
    std::ofstream odometryFile("odometry.csv", std::ios::app); // Open file in append mode
    if (!odometryFile) {
        perror("Failed to open odometry.csv for writing");
        exit(1);
    } else {
        std::cout << "[DEBUG] File odometry.csv successfully opened." << std::endl;
    }

    // If file is empty, write CSV header
    if (odometryFile.tellp() == 0) {
        odometryFile << "Timestamp,X,Y,Theta\n";
        if (odometryFile.fail()) {
            std::cerr << "[ERROR] Failed to write header to odometry.csv" << std::endl;
        } else {
            std::cout << "[DEBUG] Header written to odometry.csv" << std::endl;
        }
    }

    char buffer[BUFFER_SIZE];
    ssize_t bytesReceived;

    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate the received data
        std::cout << "[DEBUG] Received raw data: " << buffer << std::endl;

        // Parse data if structured (assume "X:Y:Theta" format for this example)
        std::istringstream stream(buffer);
        std::string segment;
        while (std::getline(stream, segment, ';')) { // Split by ';'
            std::string x, y, theta;
            std::istringstream segmentStream(segment);
            if (std::getline(segmentStream, x, ':') && std::getline(segmentStream, y, ':') && std::getline(segmentStream, theta, ':')) {
                // Get current timestamp
                std::time_t now = std::time(nullptr);
                std::tm *localTime = std::localtime(&now); // Convert to local time
                odometryFile << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "," << x << "," << y << "," << theta;
                odometryFile.flush(); // Force writing to the file

                if (odometryFile.fail()) {
                    std::cerr << "[ERROR] Failed to write data: " << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "," << x << "," << y << "," << theta << std::endl;
                } else {
                    std::cout << "[DEBUG] Wrote data to odometry.csv: " << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "," << x << "," << y << "," << theta << std::endl;
                }
            } else {
                std::cerr << "[ERROR] Malformed data segment: " << segment << std::endl;
            }
        }
    }

    if (bytesReceived < 0) {
        perror("[ERROR] recv() failed");
        exit(1);
    } else {
        std::cout << "[DEBUG] Connection closed by client." << std::endl;
    }

    odometryFile.close(); // Close the file
    std::cout << "[DEBUG] File odometry.csv closed." << std::endl;
}

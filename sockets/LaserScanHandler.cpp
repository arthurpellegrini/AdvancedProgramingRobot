#include "LaserScanHandler.h"

// Function to receive and save LaserScan data
void ReceiveAndSaveLaserScanData(int sock) {
    std::ofstream laserFile("laserscan.csv", std::ios::app); // Open file in append mode
    if (!laserFile) {
        perror("Failed to open laserscan.csv for writing");
        exit(1);
    } else {
        std::cout << "[DEBUG] File laserscan.csv successfully opened." << std::endl;
    }

    // If file is empty, write CSV header
    if (laserFile.tellp() == 0) {
        laserFile << "Timestamp,Angle,Distance\n";
        if (laserFile.fail()) {
            std::cerr << "[ERROR] Failed to write header to laserscan.csv" << std::endl;
        } else {
            std::cout << "[DEBUG] Header written to laserscan.csv" << std::endl;
        }
    }

    char buffer[BUFFER_SIZE];
    ssize_t bytesReceived;

    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate the received data
        std::cout << "[DEBUG] Received raw data: " << buffer << std::endl;

        // Parse data if structured (assume "angle:distance" format for this example)
        std::istringstream stream(buffer);
        std::string segment;
        while (std::getline(stream, segment, ';')) { // Split by ';'
            std::string angle, distance;
            std::istringstream segmentStream(segment);
            if (std::getline(segmentStream, angle, ':') && std::getline(segmentStream, distance, ':')) {
                // Get current timestamp
                std::time_t now = std::time(nullptr);
                std::tm *localTime = std::localtime(&now); // Convert to local time
                laserFile << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "," << angle << "," << distance;
                laserFile.flush(); // Force writing to the file

                if (laserFile.fail()) {
                    std::cerr << "[ERROR] Failed to write data: " << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "," << angle << "," << distance << std::endl;
                } else {
                    std::cout << "[DEBUG] Wrote data to laserscan.csv: " << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "," << angle << "," << distance << std::endl;
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

    laserFile.close(); // Close the file
    std::cout << "[DEBUG] File laserscan.csv closed." << std::endl;
}

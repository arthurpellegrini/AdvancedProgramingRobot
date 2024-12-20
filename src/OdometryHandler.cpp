#include "OdometryHandler.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib> // For system()

void ReceiveAndSaveOdometryData(int sock) {
    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);
    char dateTime[20];
    std::strftime(dateTime, sizeof(dateTime), "%Y-%m-%d_%H-%M", localTime); // Format as "YYYY-MM-DD_HH-MM"

    // Ensure the directory exists
    std::string directoryPath = "./data_test/odometry";
    std::string command = "mkdir -p " + directoryPath;
    if (system(command.c_str()) != 0) {
        perror("Failed to create directory");
        exit(1);
    }

    // Construct the filename
    std::string filename = directoryPath + "/odom_" + dateTime + ".json";

    // Open the file for writing
    std::ofstream odometryFile(filename, std::ios::app);
    if (!odometryFile) {
        perror(("Failed to open " + filename + " for writing").c_str());
        exit(1);
    } else {
        std::cout << "[DEBUG] File " << filename << " successfully opened." << std::endl;
        odometryFile << "[" << std::endl;
    }

    char buffer[BUFFER_SIZE];
    ssize_t bytesReceived;
    std::ostringstream dataStream;
    bool isReadingData = false;

    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate received data
        std::string chunk(buffer);

        // Check for data markers
        if (chunk.find("---START---") != std::string::npos) {
            isReadingData = true; // Begin reading data
            dataStream.str("");   // Clear the stream
        }
        if (isReadingData) {
            dataStream << chunk; // Append data to the stream
        }
        if (chunk.find("___END___") != std::string::npos) {
            isReadingData = false; // End reading data

            // Extract JSON string
            std::string rawData = dataStream.str();
            size_t start = rawData.find("---START---") + 11; // Skip "---START---"
            size_t end = rawData.find("___END___");
            if (start != std::string::npos && end != std::string::npos) {
                std::string jsonData = rawData.substr(start, end - start);

                // Parse the JSON
                Json::Value odometryData;
                Json::CharReaderBuilder readerBuilder;
                std::string errors;

                std::istringstream jsonStream(jsonData);
                std::cout << "[Odometry]";
                if (Json::parseFromStream(readerBuilder, jsonStream, &odometryData, &errors)) {
                    // std::cout << "[DEBUG] Parsed JSON data:\n" << odometryData.toStyledString() << std::endl;

                    // Write the parsed JSON to file
                    Json::StreamWriterBuilder writerBuilder;
                    writerBuilder["indentation"] = "    "; // Pretty-print with 4 spaces
                    odometryFile << Json::writeString(writerBuilder, odometryData) << ", " << std::endl;

                    std::cout << "[DEBUG] JSON data written to file." << std::endl;
                } else {
                    std::cerr << "[ERROR] Failed to parse JSON: " << errors << std::endl;
                }
            } else {
                std::cerr << "[ERROR] Malformed data, could not find valid JSON." << std::endl;
            }
        }
    }

    if (bytesReceived < 0) {
        perror("[ERROR] recv() failed");
    } else {
        std::cout << "[DEBUG] Connection closed by client." << std::endl;
    }

    // Close the file
    odometryFile.close();
    std::cout << "[DEBUG] File " << filename << " closed." << std::endl;
}

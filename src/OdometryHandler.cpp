#include "OdometryHandler.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib> // For system()
#include "SharedMemory.h"

/**
 * @file OdometryHandler.cpp
 * 
 * @brief Receives and processes Odometry data from a TCP socket.
 * 
 * This function continuously receives Odometry data from the specified socket,
 * processes it, and saves filtered JSON data to a file. The data is also written 
 * into shared memory for inter-process communication. The function ensures thread 
 * safety using semaphores.
 * 
 * @param sock The socket descriptor used to receive data.
 * @param shared Pointer to the shared memory structure for storing Odometry data.
 * @param odometrySemaphore Pointer to the semaphore for synchronizing shared memory access.
 * 
 * @details The received data is expected to be in a custom format, starting with `---START---` 
 * and ending with `___END___`. The content between these markers is parsed as JSON. Relevant 
 * fields such as `pose` (including `position` and `orientation`) are extracted, written to a 
 * JSON file, and stored in shared memory.
 * 
 * @note Ensure that the specified socket is connected to the correct server before calling this 
 * function. The directory `./tmp` is created to store the output file `odometry_data.json`.
 */
void ReceiveAndSaveOdometryData(int sock, SharedData* shared, sem_t* odometrySemaphore) {
    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);
    char dateTime[20];
    std::strftime(dateTime, sizeof(dateTime), "%Y-%m-%d_%H-%M", localTime); // Format as "YYYY-MM-DD_HH-MM"

    // Ensure the directory exists
    std::string directoryPath = "./tmp";
    std::string command = "mkdir -p " + directoryPath;
    if (system(command.c_str()) != 0) {
        perror("Failed to create directory");
        exit(1);
    }

    // Construct the filename
    std::string filename = directoryPath + "/odometry_data.json";

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
            if (start != std::string::npos && end != std::string::npos && start < end) {
                std::string jsonData = rawData.substr(start, end - start);

                // Parse the JSON
                Json::Value odometryData;
                Json::CharReaderBuilder readerBuilder;
                std::string errors;
                std::istringstream jsonStream(jsonData);

                if (Json::parseFromStream(readerBuilder, jsonStream, &odometryData, &errors)) {
                    Json::Value filteredData;
                    if (odometryData.isMember("pose") && odometryData["pose"].isMember("pose")) {
                        const Json::Value& pose = odometryData["pose"]["pose"];
                        if (pose.isMember("position")) {
                            filteredData["pose"]["pose"]["position"] = pose["position"];
                        }
                        if (pose.isMember("orientation")) {
                            filteredData["pose"]["pose"]["orientation"] = pose["orientation"];
                        }
                    }

                    std::ofstream odometryFile(filename, std::ios::trunc); // Overwrite the file
                    if (odometryFile) {
                        Json::StreamWriterBuilder writerBuilder;
                        writerBuilder["indentation"] = "    ";
                        odometryFile << Json::writeString(writerBuilder, filteredData) << std::endl;
                        odometryFile.close();
                        std::cout << "[DEBUG] JSON data written to file." << std::endl;
                    } else {
                        std::cerr << "[ERROR] Failed to open " << filename << " for writing." << std::endl;
                    }

                    if (!jsonData.empty()) {
                        sem_wait(odometrySemaphore);
                        strncpy(shared->odometryData, jsonData.c_str(), sizeof(shared->odometryData) - 1);
                        shared->odometryData[sizeof(shared->odometryData) - 1] = '\0';
                        sem_post(odometrySemaphore);
                    }
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
}

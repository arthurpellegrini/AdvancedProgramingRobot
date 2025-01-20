#include "LaserScanHandler.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib> // For system()
#include "SharedMemory.h"

/**
 * @brief Receives and processes LaserScan data from a TCP socket.
 * 
 * This function continuously receives LaserScan data from the specified socket, processes it,
 * and saves filtered JSON data to a file. The data is also written into shared memory for
 * inter-process communication. The function ensures thread safety using semaphores.
 * 
 * @param sock The socket descriptor used to receive data.
 * @param shared Pointer to the shared memory structure for storing LaserScan data.
 * @param laserSemaphore Pointer to the semaphore for synchronizing shared memory access.
 * 
 * @details The received data is expected to be in a custom format, starting with `---START---` 
 * and ending with `___END___`. The content between these markers is parsed as JSON. Relevant 
 * fields such as `angle_increment` and `ranges` are extracted, written to a JSON file, and 
 * stored in shared memory.
 * 
 * @note Ensure that the specified socket is connected to the correct server before calling 
 * this function. The directory `./tmp` is created to store the output file `laser_data.json`.
 */
void ReceiveAndSaveLaserScanData(int sock, SharedData* shared, sem_t* laserSemaphore) {
    // Create the directory for storing laser data files
    std::string directoryPath = "./tmp";
    std::string command = "mkdir -p " + directoryPath;
    if (system(command.c_str()) != 0) {
        perror("Failed to create directory");
        exit(1);
    }

    std::string filename = directoryPath + "/laser_data.json";

    char buffer[BUFFER_SIZE];
    ssize_t bytesReceived;
    std::ostringstream dataStream;
    bool isReadingData = false;

    // Continuously receive and process data from the socket
    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate received data
        std::string chunk(buffer);

        // Check for start and end markers
        if (chunk.find("---START---") != std::string::npos) {
            isReadingData = true;
            dataStream.str("");
        }
        if (isReadingData) {
            dataStream << chunk;
        }
        if (chunk.find("___END___") != std::string::npos) {
            isReadingData = false;

            // Extract the JSON data between markers
            std::string rawData = dataStream.str();
            size_t start = rawData.find("---START---") + 11;
            size_t end = rawData.find("___END___");

            if (start != std::string::npos && end != std::string::npos && start < end) {
                std::string jsonData = rawData.substr(start, end - start);

                // Parse the JSON data
                Json::Value laserScanData;
                Json::CharReaderBuilder readerBuilder;
                std::string errors;
                std::istringstream jsonStream(jsonData);

                if (Json::parseFromStream(readerBuilder, jsonStream, &laserScanData, &errors)) {
                    // Filter and save relevant JSON data
                    Json::Value filteredData;
                    if (laserScanData.isMember("angle_increment")) {
                        filteredData["angle_increment"] = laserScanData["angle_increment"];
                    }
                    if (laserScanData.isMember("ranges")) {
                        filteredData["ranges"] = laserScanData["ranges"];
                    }

                    std::ofstream laserFile(filename, std::ios::trunc); // Overwrite the file
                    if (laserFile) {
                        Json::StreamWriterBuilder writerBuilder;
                        writerBuilder["indentation"] = "    ";
                        laserFile << Json::writeString(writerBuilder, filteredData) << std::endl;
                        laserFile.close();
                        std::cout << "[DEBUG] JSON data written to file." << std::endl;
                    } else {
                        std::cerr << "[ERROR] Failed to open " << filename << " for writing." << std::endl;
                    }

                    // Store the raw JSON data in shared memory
                    if (!jsonData.empty()) {
                        sem_wait(laserSemaphore);
                        strncpy(shared->laserScanData, jsonData.c_str(), sizeof(shared->laserScanData) - 1);
                        shared->laserScanData[sizeof(shared->laserScanData) - 1] = '\0';
                        sem_post(laserSemaphore);
                    }
                } else {
                    std::cerr << "[ERROR] Failed to parse JSON: " << errors << std::endl;
                }
            } else {
                std::cerr << "[ERROR] Malformed data, could not find valid JSON." << std::endl;
            }
        }
    }

    // Handle errors or connection closure
    if (bytesReceived < 0) {
        perror("[ERROR] recv() failed");
    } else {
        std::cout << "[DEBUG] Connection closed by client." << std::endl;
    }
}

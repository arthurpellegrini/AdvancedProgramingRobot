#include "LaserScanHandler.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib> // For system()
#include "SharedMemory.h"

void ReceiveAndSaveLaserScanData(int sock, SharedData* shared, sem_t* laserSemaphore) {
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

    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate received data
        std::string chunk(buffer);

        if (chunk.find("---START---") != std::string::npos) {
            isReadingData = true;
            dataStream.str("");
        }
        if (isReadingData) {
            dataStream << chunk;
        }
        if (chunk.find("___END___") != std::string::npos) {
            isReadingData = false;

            std::string rawData = dataStream.str();
            size_t start = rawData.find("---START---") + 11;
            size_t end = rawData.find("___END___");
            
            if (start != std::string::npos && end != std::string::npos && start < end) {
                std::string jsonData = rawData.substr(start, end - start);
                
                Json::Value laserScanData;
                Json::CharReaderBuilder readerBuilder;
                std::string errors;
                std::istringstream jsonStream(jsonData);

                if (Json::parseFromStream(readerBuilder, jsonStream, &laserScanData, &errors)) {
                    Json::Value filteredData;
                    if (laserScanData.isMember("angle_increment")) {
                        filteredData["angle_increment"] = laserScanData["angle_increment"];
                    }
                    if (laserScanData.isMember("ranges")) {
                        filteredData["ranges"] = laserScanData["ranges"];
                    }
                    
                    std::ofstream laserFile(filename, std::ios::trunc); // Überschreiben der Datei
                    if (laserFile) {
                        Json::StreamWriterBuilder writerBuilder;
                        writerBuilder["indentation"] = "    ";
                        laserFile << Json::writeString(writerBuilder, filteredData) << std::endl;
                        laserFile.close();
                        std::cout << "[DEBUG] JSON data written to file." << std::endl;
                    } else {
                        std::cerr << "[ERROR] Failed to open " << filename << " for writing." << std::endl;
                    }

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

    if (bytesReceived < 0) {
        perror("[ERROR] recv() failed");
    } else {
        std::cout << "[DEBUG] Connection closed by client." << std::endl;
    }
}

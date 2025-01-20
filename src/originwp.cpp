#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>          // For matrix operations (used in localization)
#include <pcl/point_cloud.h>   // For handling point clouds
#include <pcl/io/pcd_io.h>     // For saving/loading point cloud maps
#include <pcl/registration/icp.h> // For ICP-based localization

// Placeholder for SLAM Library (e.g., GMapping or Cartographer)
// #include <YourSLAMLibrary.h>

// Map Utilities
#include <opencv2/opencv.hpp> // For occupancy grid map handling
#include <yaml-cpp/yaml.h>    // For parsing map metadata

// Define constants
const std::string MAP_IMAGE = "map.pgm"; // Occupancy grid map
const std::string MAP_METADATA = "map.yaml";

// Struct to hold the robot's pose
struct Pose {
    double x, y, theta; // Position (x, y) and orientation (theta in radians)
};

// Function to load the map from files
cv::Mat loadOccupancyGrid(const std::string& mapImage, const std::string& metadataFile) {
    // Parse YAML metadata for resolution and origin
    YAML::Node metadata = YAML::LoadFile(metadataFile);
    double resolution = metadata["resolution"].as<double>();
    std::vector<double> origin = metadata["origin"].as<std::vector<double>>();

    std::cout << "Map Resolution: " << resolution << " meters/pixel" << std::endl;
    std::cout << "Map Origin: (" << origin[0] << ", " << origin[1] << ")" << std::endl;

    // Load the occupancy grid map as a grayscale image
    cv::Mat map = cv::imread(mapImage, cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        std::cerr << "Error: Unable to load map image!" << std::endl;
        exit(EXIT_FAILURE);
    }
    return map;
}

// Function to perform SLAM and save the generated map
void runSLAM() {
    std::cout << "Running SLAM..." << std::endl;

    // Placeholder for integrating your SLAM library
    // Initialize SLAM, process sensor data, and build the map
    // YourSLAMLibrary slam;
    // slam.process(sensorData);

    // Save the generated map (as an example, use a placeholder image)
    cv::Mat dummyMap(500, 500, CV_8UC1, cv::Scalar(255)); // Dummy map
    cv::imwrite(MAP_IMAGE, dummyMap);

    // Save map metadata
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "resolution" << YAML::Value << 0.05; // Example resolution
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow << std::vector<double>{0.0, 0.0, 0.0};
    out << YAML::Key << "width" << YAML::Value << dummyMap.cols;
    out << YAML::Key << "height" << YAML::Value << dummyMap.rows;
    out << YAML::EndMap;

    std::ofstream fout(MAP_METADATA);
    fout << out.c_str();
    fout.close();
    std::cout << "Map and metadata saved." << std::endl;
}

// Function to localize the robot in the saved map
Pose localizeRobot(const cv::Mat& map) {
    std::cout << "Localizing robot..." << std::endl;

    // Placeholder for localization
    // Use ICP or Monte Carlo Localization (MCL) to find the pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>());

    // Load mapCloud and currentScan (replace with real data)
    pcl::io::loadPCDFile("map_cloud.pcd", *mapCloud);
    pcl::io::loadPCDFile("current_scan.pcd", *currentScan);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(currentScan);
    icp.setInputTarget(mapCloud);

    pcl::PointCloud<pcl::PointXYZ> alignedScan;
    icp.align(alignedScan);

    if (icp.hasConverged()) {
        std::cout << "ICP converged. Fitness score: " << icp.getFitnessScore() << std::endl;
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        double x = transformation(0, 3);
        double y = transformation(1, 3);
        double theta = std::atan2(transformation(1, 0), transformation(0, 0));
        return {x, y, theta};
    } else {
        std::cerr << "ICP failed to converge." << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Main Function
int main() {
    std::cout << "Robot Localization and Mapping System" << std::endl;

    // Check if mapping is needed
    bool needsMapping = true;

    if (needsMapping) {
        // Run SLAM to generate the map
        runSLAM();
    }

    // Load the saved map
    cv::Mat map = loadOccupancyGrid(MAP_IMAGE, MAP_METADATA);

    // Perform localization
    Pose robotPose = localizeRobot(map);
    std::cout << "Robot Pose: x = " << robotPose.x << ", y = " << robotPose.y
              << ", theta = " << robotPose.theta << " radians" << std::endl;

    return 0;
}

#include "HandleLaserScanData.h"

void handleLaserScanData(const char* data, size_t length) {
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string errs;
    std::istringstream sstream(std::string(data, length));

    if (!Json::parseFromStream(readerBuilder, sstream, &root, &errs)) {
        ROS_ERROR("Failed to parse LaserScan data: %s", errs.c_str());
        return;
    }

    sensor_msgs::LaserScan laserScan;
    try {
        laserScan.header.seq = root["header"]["seq"].asUInt();
        laserScan.header.stamp.sec = root["header"]["stamp"]["secs"].asInt();
        laserScan.header.stamp.nsec = root["header"]["stamp"]["nsecs"].asInt();
        laserScan.header.frame_id = root["header"]["frame_id"].asString();
        laserScan.angle_min = root["angle_min"].asFloat();
        laserScan.angle_max = root["angle_max"].asFloat();
        laserScan.angle_increment = root["angle_increment"].asFloat();
        laserScan.time_increment = root["time_increment"].asFloat();
        laserScan.scan_time = root["scan_time"].asFloat();
        laserScan.range_min = root["range_min"].asFloat();
        laserScan.range_max = root["range_max"].asFloat();

        const Json::Value ranges = root["ranges"];
        for (const auto& range : ranges) {
            laserScan.ranges.push_back(range.asFloat());
        }

        const Json::Value intensities = root["intensities"];
        for (const auto& intensity : intensities) {
            laserScan.intensities.push_back(intensity.asFloat());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while parsing LaserScan data: %s", e.what());
        return;
    }
    
    // Process the laser scan data
    ROS_INFO("Received LaserScan data with range size: %lu", laserScan.ranges.size());
}
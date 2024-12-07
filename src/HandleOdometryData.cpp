#include "HandleOdometryData.h"

void handleOdometryData(const char* data, size_t length) {
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string errs;
    std::istringstream sstream(std::string(data, length));

    if (!Json::parseFromStream(readerBuilder, sstream, &root, &errs)) {
        ROS_ERROR("Failed to parse Odometry data: %s", errs.c_str());
        return;
    }

    nav_msgs::Odometry odometry;
    try {
        odometry.header.seq = root["header"]["seq"].asUInt();
        odometry.header.stamp.sec = root["header"]["stamp"]["secs"].asInt();
        odometry.header.stamp.nsec = root["header"]["stamp"]["nsecs"].asInt();
        odometry.header.frame_id = root["header"]["frame_id"].asString();
        odometry.child_frame_id = root["child_frame_id"].asString();

        odometry.pose.pose.position.x = root["pose"]["pose"]["position"]["x"].asFloat();
        odometry.pose.pose.position.y = root["pose"]["pose"]["position"]["y"].asFloat();
        odometry.pose.pose.position.z = root["pose"]["pose"]["position"]["z"].asFloat();
        odometry.pose.pose.orientation.x = root["pose"]["pose"]["orientation"]["x"].asFloat();
        odometry.pose.pose.orientation.y = root["pose"]["pose"]["orientation"]["y"].asFloat();
        odometry.pose.pose.orientation.z = root["pose"]["pose"]["orientation"]["z"].asFloat();
        odometry.pose.pose.orientation.w = root["pose"]["pose"]["orientation"]["w"].asFloat();

        const Json::Value covariance = root["pose"]["covariance"];
        for (Json::ArrayIndex i = 0; i < covariance.size(); ++i) {
            odometry.pose.covariance[i] = covariance[i].asFloat();
        }

        odometry.twist.twist.linear.x = root["twist"]["twist"]["linear"]["x"].asFloat();
        odometry.twist.twist.linear.y = root["twist"]["twist"]["linear"]["y"].asFloat();
        odometry.twist.twist.linear.z = root["twist"]["twist"]["linear"]["z"].asFloat();
        odometry.twist.twist.angular.x = root["twist"]["twist"]["angular"]["x"].asFloat();
        odometry.twist.twist.angular.y = root["twist"]["twist"]["angular"]["y"].asFloat();
        odometry.twist.twist.angular.z = root["twist"]["twist"]["angular"]["z"].asFloat();

        const Json::Value twist_covariance = root["twist"]["covariance"];
        for (Json::ArrayIndex i = 0; i < twist_covariance.size(); ++i) {
            odometry.twist.covariance[i] = twist_covariance[i].asFloat();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while parsing Odometry data: %s", e.what());
        return;
    }
    
    // Process the odometry data
    ROS_INFO("Received Odometry data with position x: %f, y: %f, z: %f", 
             odometry.pose.pose.position.x, 
             odometry.pose.pose.position.y, 
             odometry.pose.pose.position.z);
} 

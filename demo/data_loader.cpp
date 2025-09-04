#include "data_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

namespace dogm {

RealDataLoader::RealDataLoader(const std::string& csv_filepath) {
    parse(csv_filepath);
}

void RealDataLoader::parse(const std::string& csv_filepath) {
    std::ifstream file(csv_filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Could not open data file " + csv_filepath);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        double timestamp;
        std::string data_type;

        std::getline(ss, token, ',');
        timestamp = std::stod(token);
        std::getline(ss, token, ',');
        data_type = token;

        auto& frame = data_map[timestamp];
        frame.timestamp = timestamp;

        if (data_type == "odom") {
            std::getline(ss, token, ','); frame.ego_pose.x() = std::stof(token);
            std::getline(ss, token, ','); frame.ego_pose.y() = std::stof(token);
            std::getline(ss, token, ','); frame.ego_yaw = std::stof(token);
        } else if (data_type == "lidar") {
            while (std::getline(ss, token, ',')) {
                float angle = std::stof(token);
                if (std::getline(ss, token, ',')) {
                    float range = std::stof(token);
                    frame.lidar.angles.push_back(angle);
                    frame.lidar.ranges.push_back(range);
                }
            }
        } else if (data_type == "radar") {
            RadarDetection detection;
            std::getline(ss, token, ','); detection.position.x() = std::stof(token);
            std::getline(ss, token, ','); detection.position.y() = std::stof(token);
            std::getline(ss, token, ','); detection.radial_velocity = std::stof(token);
            frame.radar.push_back(detection);
        }
    }

    for(auto const& [key, val] : data_map) {
        timestamp_keys.push_back(key);
    }
}

bool RealDataLoader::hasNextFrame() const {
    return current_frame_index < timestamp_keys.size();
}

SensorFrame RealDataLoader::getNextFrame() {
    if (!hasNextFrame()) {
        throw std::out_of_range("No more frames to load.");
    }
    double key = timestamp_keys[current_frame_index];
    current_frame_index++;
    return data_map[key];
}

} // namespace dogm
#include "data_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <map>
#include <cmath>
#include <set> // std::set을 위해 추가

namespace dogm {

RealDataLoader::RealDataLoader(const std::string& data_path) {
    std::cout << "Loading data from path: " << data_path << std::endl;
    
    std::string lidar_file = data_path + "/LiDARMap_v2.txt";
    std::string radar_file = data_path + "/RadarMap_v2.txt";
    
    loadLidarData(lidar_file);
    loadRadarData(radar_file);
    
    // 공통 타임스탬프 찾기
    std::set<double> common_timestamps;
    for (const auto& pair : lidar_data) {
        if (radar_data.count(pair.first)) {
            common_timestamps.insert(pair.first);
        }
    }
    
    timestamps.assign(common_timestamps.begin(), common_timestamps.end());
    total_frames = timestamps.size();
    
    if (total_frames == 0) {
        throw std::runtime_error("Error: No matching timestamps found");
    }
    std::cout << "Found " << total_frames << " frames." << std::endl;
}

void RealDataLoader::loadLidarData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open LiDAR file: " + filename);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double timestamp, x, y, intensity;
        if (ss >> timestamp >> x >> y >> intensity) {
            double angle = std::atan2(y, x);
            double range = std::sqrt(x*x + y*y);
            lidar_data[timestamp].angles.push_back(angle);
            lidar_data[timestamp].ranges.push_back(range);
        }
    }
}

void RealDataLoader::loadRadarData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open Radar file: " + filename);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double timestamp, x, y, velocity, snr;
        if (ss >> timestamp >> x >> y >> velocity >> snr) {
            RadarDetection detection;
            detection.position = Eigen::Vector2f(x, y);
            detection.radial_velocity = velocity;
            detection.snr = snr; // snr 값을 저장하도록 수정
            radar_data[timestamp].push_back(detection);
        }
    }
}

bool RealDataLoader::hasNextFrame() const {
    return current_frame_index < total_frames;
}

SensorFrame RealDataLoader::getNextFrame() {
    if (!hasNextFrame()) {
        throw std::out_of_range("No more frames to load.");
    }

    SensorFrame frame;
    double timestamp = timestamps[current_frame_index];
    frame.timestamp = timestamp;

    // Load Lidar Data
    if (lidar_data.count(timestamp)) {
        frame.lidar = lidar_data[timestamp];
    }

    // Load Radar Data
    if (radar_data.count(timestamp)) {
        frame.radar = radar_data[timestamp];
    }

    // Ego pose (고정값 사용)
    frame.ego_pose = {10.0f, 1.0f};
    frame.ego_yaw = M_PI / 2.0;

    current_frame_index++;
    return frame;
}

} // namespace dogm
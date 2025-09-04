#include "data_loader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <glob.h>

namespace dogm {

// Utility to get sorted file list from a pattern
std::vector<std::string> glob_files(const std::string& pattern) {
    glob_t glob_result;
    glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    std::vector<std::string> files;
    for(unsigned int i=0; i<glob_result.gl_pathc; ++i) {
        files.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    std::sort(files.begin(), files.end());
    return files;
}

DataLoader::DataLoader(const std::string& path) : data_path(path), current_frame_index(0) {
    lidar_files = glob_files(data_path + "LiDARMap_v*.txt");
    radar_files = glob_files(data_path + "RadarMap_v*.txt");
    total_frames = std::min(lidar_files.size(), radar_files.size());

    if (total_frames == 0) {
        std::cerr << "Error: No data files found in " << data_path << std::endl;
    }
}

bool DataLoader::hasNextFrame() const {
    return current_frame_index < total_frames;
}

SensorFrame DataLoader::getNextFrame() {
    if (!hasNextFrame()) {
        throw std::out_of_range("No more frames to load.");
    }

    SensorFrame frame;
    frame.timestamp = current_frame_index * 0.1; // Assume 10Hz

    loadLidarData(lidar_files[current_frame_index], frame.lidar);
    loadRadarData(radar_files[current_frame_index], frame.radar);

    // For simplicity, ego pose is static in this loader
    frame.ego_pose = Vec2(1.5f, 0.5f); // Robot at the bottom center
    frame.ego_yaw = M_PI / 2.0f; // Facing up

    current_frame_index++;
    return frame;
}

bool DataLoader::loadLidarData(const std::string& filename, LidarMeasurement& lidar) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open LiDAR file " << filename << std::endl;
        return false;
    }

    lidar.ranges.clear();
    lidar.angles.clear();

    std::string line;
    // The first line is number of points, we can ignore it and read until EOF
    std::getline(file, line); 
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        float angle, range;
        if (ss >> angle >> range) {
            lidar.angles.push_back(angle);
            lidar.ranges.push_back(range);
        }
    }
    return true;
}

bool DataLoader::loadRadarData(const std::string& filename, RadarMeasurement& radar) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open Radar file " << filename << std::endl;
        return false;
    }
    
    radar.detections.clear();

    std::string line;
    // The first line is number of points, we can ignore it
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        RadarDetection detection;
        if (ss >> detection.position.x() >> detection.position.y() >> detection.radial_velocity >> detection.intensity) {
            radar.detections.push_back(detection);
        }
    }
    return true;
}

} // namespace dogm
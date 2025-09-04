#include "data_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <windows.h> // For Windows-specific file finding
#include <vector>
#include <algorithm>

namespace dogm {

// Windows 환경에서 파일 목록을 가져오는 유틸리티 함수
std::vector<std::string> get_files_in_directory(const std::string& directory, const std::string& filter) {
    std::vector<std::string> files;
    std::string search_path = directory + "/" + filter;
    WIN32_FIND_DATAA find_data;
    HANDLE h_find = FindFirstFileA(search_path.c_str(), &find_data);

    if (h_find != INVALID_HANDLE_VALUE) {
        do {
            files.push_back(directory + "/" + std::string(find_data.cFileName));
        } while (FindNextFileA(h_find, &find_data));
        FindClose(h_find);
    }
    std::sort(files.begin(), files.end());
    return files;
}


RealDataLoader::RealDataLoader(const std::string& data_path) {
    std::cout << "Loading data from path: " << data_path << std::endl;
    lidar_files = get_files_in_directory(data_path, "lidar_frame_*.txt");
    radar_files = get_files_in_directory(data_path, "radar_frame_*.txt");
    total_frames = std::min(lidar_files.size(), radar_files.size());

    if (total_frames == 0) {
        throw std::runtime_error("Error: No data files found in " + data_path);
    }
     std::cout << "Found " << total_frames << " frames." << std::endl;
}

bool RealDataLoader::hasNextFrame() const {
    return current_frame_index < total_frames;
}

SensorFrame RealDataLoader::getNextFrame() {
    if (!hasNextFrame()) {
        throw std::out_of_range("No more frames to load.");
    }

    SensorFrame frame;
    frame.timestamp = static_cast<double>(current_frame_index) * 0.1; // 10Hz 가정

    // Load Lidar Data
    std::ifstream lidar_file(lidar_files[current_frame_index]);
    if(lidar_file.is_open()){
        std::string line;
        std::getline(lidar_file, line); // Skip header
        while (std::getline(lidar_file, line)) {
            std::stringstream ss(line);
            float angle, range;
            if (ss >> angle >> range) {
                frame.lidar.angles.push_back(angle);
                frame.lidar.ranges.push_back(range);
            }
        }
    } else {
         std::cerr << "Warning: Could not open LiDAR file " << lidar_files[current_frame_index] << std::endl;
    }


    // Load Radar Data
    std::ifstream radar_file(radar_files[current_frame_index]);
     if(radar_file.is_open()){
        std::string line;
        std::getline(radar_file, line); // Skip header
        while (std::getline(radar_file, line)) {
            std::stringstream ss(line);
            RadarDetection detection;
            if (ss >> detection.position.x() >> detection.position.y() >> detection.radial_velocity) {
                frame.radar.push_back(detection);
            }
        }
    } else {
         std::cerr << "Warning: Could not open Radar file " << radar_files[current_frame_index] << std::endl;
    }


    // Odom은 예시로 고정값 사용. 실제 데이터가 있다면 파싱 로직 추가 필요.
    frame.ego_pose = {10.0f, 1.0f}; // 그리드 중앙 하단
    frame.ego_yaw = M_PI / 2.0;   // 위쪽 방향

    current_frame_index++;
    return frame;
}

} // namespace dogm
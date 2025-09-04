#include "data_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <windows.h> // Windows API 헤더 포함
#include <vector>
#include <algorithm>
#include <string>

namespace dogm {

// Windows 환경에서 특정 패턴의 파일 목록을 가져오는 유틸리티 함수
std::vector<std::string> get_files_in_directory(const std::string& directory, const std::string& filter) {
    std::vector<std::string> files;
    std::string search_path = directory + "/" + filter;
    WIN32_FIND_DATAA find_data;
    HANDLE h_find = FindFirstFileA(search_path.c_str(), &find_data);

    if (h_find != INVALID_HANDLE_VALUE) {
        do {
            // 디렉토리는 제외하고 파일만 추가
            if (!(find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                files.push_back(directory + "/" + std::string(find_data.cFileName));
            }
        } while (FindNextFileA(h_find, &find_data));
        FindClose(h_find);
    }
    // Windows의 FindFirstFile은 순서를 보장하지 않으므로, 파일 이름을 기준으로 정렬합니다.
    std::sort(files.begin(), files.end());
    return files;
}

RealDataLoader::RealDataLoader(const std::string& data_path) : current_frame_index(0) {
    std::cout << "Loading data from path: " << data_path << std::endl;
    lidar_files = get_files_in_directory(data_path, "lidar_frame_*.txt");
    radar_files = get_files_in_directory(data_path, "radar_frame_*.txt");
    total_frames = std::min(lidar_files.size(), radar_files.size());

    if (total_frames == 0) {
        throw std::runtime_error("Error: No data files found in " + data_path);
    }
    std::cout << "Found " << total_frames << " synchronized frames." << std::endl;
}

bool RealDataLoader::hasNextFrame() const {
    return current_frame_index < total_frames;
}

SensorFrame RealDataLoader::getNextFrame() {
    if (!hasNextFrame()) {
        throw std::out_of_range("No more frames to load.");
    }

    SensorFrame frame;
    frame.timestamp = static_cast<double>(current_frame_index) * 0.1; // 10Hz 데이터로 가정

    // Lidar 데이터 로드
    std::ifstream lidar_file(lidar_files[current_frame_index]);
    if (lidar_file.is_open()) {
        std::string line;
        std::getline(lidar_file, line); // 헤더 라인 스킵
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

    // Radar 데이터 로드
    std::ifstream radar_file(radar_files[current_frame_index]);
    if (radar_file.is_open()) {
        std::string line;
        std::getline(radar_file, line); // 헤더 라인 스킵
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

    // Odometry 데이터는 실제 파일이 없으므로 예시로 고정값을 사용합니다.
    // 그리드의 중앙 하단에서 위쪽(Y+ 방향)을 바라보는 로봇을 가정합니다.
    // 그리드 크기가 20m, 해상도가 0.2m/cell 이므로 그리드 셀 개수는 100x100.
    // 로봇의 위치 (10m, 2m) -> 셀 좌표 (50, 90)
    frame.ego_pose = {10.0f, 2.0f};
    frame.ego_yaw = M_PI / 2.0;

    current_frame_index++;
    return frame;
}

} // namespace dogm

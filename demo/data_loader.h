#pragma once

#include "dogm/dogm_types.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace dogm {

class RealDataLoader {
public:
    explicit RealDataLoader(const std::string& data_path);
    
    bool hasNextFrame() const;
    SensorFrame getNextFrame();
    
    size_t getCurrentFrameIndex() const { return current_frame_index; }
    size_t getTotalFrames() const { return total_frames; }

private:
    void loadLidarData(const std::string& filename);
    void loadRadarData(const std::string& filename);
    
    std::map<double, LidarScan> lidar_data;
    std::map<double, std::vector<RadarDetection>> radar_data;
    std::vector<double> timestamps;
    
    size_t current_frame_index = 0;
    size_t total_frames = 0;
};

} // namespace dogm
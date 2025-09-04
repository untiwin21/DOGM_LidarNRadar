#pragma once

#include "dogm/dogm_types.h"
#include <string>
#include <vector>

namespace dogm {

class DataLoader {
public:
    DataLoader(const std::string& data_path);
    
    bool hasNextFrame() const;
    SensorFrame getNextFrame();
    int getCurrentFrameIndex() const { return current_frame_index; }
    
private:
    bool loadLidarData(const std::string& filename, LidarMeasurement& lidar);
    bool loadRadarData(const std::string& filename, RadarMeasurement& radar);
    
    std::string data_path;
    int current_frame_index;
    int total_frames;
};

} // namespace dogm

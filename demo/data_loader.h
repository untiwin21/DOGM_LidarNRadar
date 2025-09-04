#pragma once

#include "dogm/dogm_types.h"
#include <string>
#include <vector>
#include <map>

namespace dogm {

class RealDataLoader {
public:
    RealDataLoader(const std::string& csv_filepath);

    bool hasNextFrame() const;
    SensorFrame getNextFrame();
    size_t getTotalFrames() const { return timestamp_keys.size(); }
    int getCurrentFrameIndex() const { return current_frame_index; }

private:
    void parse(const std::string& csv_filepath);

    std::map<double, SensorFrame> data_map;
    std::vector<double> timestamp_keys;
    int current_frame_index = 0;
};

} // namespace dogm
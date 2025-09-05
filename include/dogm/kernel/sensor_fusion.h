#pragma once

#include "dogm/dogm_types.h"
#include <vector>

namespace dogm {
namespace kernel {

// Lidar와 Radar 데이터를 모두 포함하는 SensorFrame을 인자로 받도록 하고, ego_pose, ego_yaw 추가
void fuseAndCreateMeasurementGrid(std::vector<MeasurementCell>& meas_cells,
                                 const SensorFrame& frame,
                                 int grid_size, float resolution,
                                 const Vec2& ego_pose, float ego_yaw);

} // namespace kernel
} // namespace dogm
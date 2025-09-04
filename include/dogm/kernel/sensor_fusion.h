#pragma once

#include "dogm/dogm_types.h"
#include <vector>

namespace dogm {
namespace kernel {

void fuseAndCreateMeasurementGrid(std::vector<MeasurementCell>& meas_cells,
                                 const LidarMeasurement& lidar,
                                 const RadarMeasurement& radar,
                                 int grid_size, float resolution);

} // namespace kernel
} // namespace dogm

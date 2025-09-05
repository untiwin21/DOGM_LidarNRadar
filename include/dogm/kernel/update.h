#pragma once

#include "dogm/dogm_types.h"
#include "dogm/common.h"

namespace dogm {
namespace kernel {

void particleToGrid(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                    std::vector<float>& weight_array);

// 'const ParticlesSoA& particles' 인자 제거
void updateOccupancy(std::vector<GridCell>& grid_cells,
                     const std::vector<float>& weight_array,
                     const std::vector<MeasurementCell>& meas_cells,
                     std::vector<float>& born_masses_array,
                     const DOGM::Params& params, float dt);

// 'const Vec2& ego_pose' 인자 추가
void updatePersistent(ParticlesSoA& particles, const std::vector<MeasurementCell>& meas_cells,
                      std::vector<GridCell>& grid_cells, std::vector<float>& weight_array,
                      const Vec2& ego_pose);

void computeStatisticalMoments(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                               const std::vector<float>& weight_array);

} // namespace kernel
} // namespace dogm
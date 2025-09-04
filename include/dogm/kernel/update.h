#pragma once

#include "dogm/dogm_types.h"
#include "dogm/common.h"

namespace dogm {
namespace kernel {

void particleToGrid(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                    std::vector<float>& weight_array);

void updateOccupancy(std::vector<GridCell>& grid_cells, const ParticlesSoA& particles,
                     const std::vector<float>& weight_array,
                     const std::vector<MeasurementCell>& meas_cells,
                     std::vector<float>& born_masses_array,
                     const DOGM::Params& params, float dt);

void updatePersistent(ParticlesSoA& particles, const std::vector<MeasurementCell>& meas_cells,
                      std::vector<GridCell>& grid_cells, std::vector<float>& weight_array);

void computeStatisticalMoments(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                               const std::vector<float>& weight_array);

} // namespace kernel
} // namespace dogm

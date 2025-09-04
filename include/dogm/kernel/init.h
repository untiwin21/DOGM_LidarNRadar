#pragma once

#include "dogm/dogm_types.h"
#include "dogm/common.h"

namespace dogm {
namespace kernel {

void initGridCells(std::vector<GridCell>& grid_cells, std::vector<MeasurementCell>& meas_cells);

void initParticles(ParticlesSoA& particles, RandomGenerator& rng, float max_velocity, int grid_size);

void initNewParticles(ParticlesSoA& birth_particles, const std::vector<GridCell>& grid_cells,
                      const std::vector<MeasurementCell>& meas_cells,
                      const std::vector<float>& born_masses_array, RandomGenerator& rng,
                      const DOGM::Params& params, int grid_size);

} // namespace kernel
} // namespace dogm

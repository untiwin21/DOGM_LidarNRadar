#pragma once

#include "dogm/dogm_types.h"
#include "dogm/common.h"

namespace dogm {
namespace kernel {

void resample(const ParticlesSoA& particles, ParticlesSoA& particles_next,
              const ParticlesSoA& birth_particles,
              const std::vector<float>& weight_array,
              const std::vector<float>& birth_weight_array,
              RandomGenerator& rng, const DOGM::Params& params);

} // namespace kernel
} // namespace dogm

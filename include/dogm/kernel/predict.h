#pragma once

#include "dogm/dogm_types.h"
#include "dogm/common.h"

namespace dogm {
namespace kernel {

void predict(ParticlesSoA& particles, RandomGenerator& rng, const DOGM::Params& params, int grid_size, float dt);

} // namespace kernel
} // namespace dogm

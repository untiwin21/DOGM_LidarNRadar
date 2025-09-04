#include "dogm/kernel/resampling.h"
#include <vector>
#include <numeric>

namespace dogm {
namespace kernel {

void resample(const ParticlesSoA& particles, ParticlesSoA& particles_next,
              const ParticlesSoA& birth_particles,
              const std::vector<float>& weight_array,
              const std::vector<float>& birth_weight_array,
              RandomGenerator& rng, const DOGM::Params& params) {

    size_t persistent_count = particles.size();
    size_t birth_count = birth_particles.size();
    size_t total_count = persistent_count + birth_count;

    std::vector<float> joint_weights;
    joint_weights.reserve(total_count);
    joint_weights.insert(joint_weights.end(), weight_array.begin(), weight_array.end());
    joint_weights.insert(joint_weights.end(), birth_weight_array.begin(), birth_weight_array.end());

    std::vector<float> accum_weights;
    accumulate(joint_weights, accum_weights);
    
    float total_weight = accum_weights.empty() ? 0.0f : accum_weights.back();

    if (total_weight <= 0.0f) {
        // Failsafe: if all weights are zero, reinitialize
        kernel::initParticles(particles_next, rng, params.init_max_velocity, static_cast<int>(sqrt(params.size / params.resolution)));
        return;
    }

    float new_weight = total_weight / particles_next.size();

    // Multinomial resampling
    #pragma omp parallel for
    for (size_t i = 0; i < particles_next.size(); ++i) {
        float r = rng.uniform(0.0f, total_weight);
        
        auto it = std::lower_bound(accum_weights.begin(), accum_weights.end(), r);
        int idx = std::distance(accum_weights.begin(), it);
        
        if (idx < persistent_count) {
            particles_next.state[i] = particles.state[idx];
            particles_next.associated[i] = particles.associated[idx];
        } else {
            int birth_idx = idx - persistent_count;
            particles_next.state[i] = birth_particles.state[birth_idx];
            particles_next.associated[i] = birth_particles.associated[birth_idx];
        }
        
        particles_next.weight[i] = new_weight;
        
        // Grid cell idx will be recalculated in predict step
        particles_next.grid_cell_idx[i] = 0; 
    }
}

} // namespace kernel
} // namespace dogm

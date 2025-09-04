#include "dogm/kernel/predict.h"

namespace dogm {
namespace kernel {

void predict(ParticlesSoA& particles, RandomGenerator& rng, const DOGM::Params& params, int grid_size, float dt) {
    
    #pragma omp parallel for
    for (size_t i = 0; i < particles.size(); ++i) {
        // State transition
        particles.state[i][0] += particles.state[i][2] * dt; // x += vx * dt
        particles.state[i][1] += particles.state[i][3] * dt; // y += vy * dt

        // Add process noise
        particles.state[i][0] += rng.normal(0.0f, params.stddev_process_noise_position);
        particles.state[i][1] += rng.normal(0.0f, params.stddev_process_noise_position);
        particles.state[i][2] += rng.normal(0.0f, params.stddev_process_noise_velocity);
        particles.state[i][3] += rng.normal(0.0f, params.stddev_process_noise_velocity);
        
        // Update weight
        particles.weight[i] *= params.persistence_prob;
        
        // Update grid cell index and check boundaries
        const auto& state = particles.state[i];
        float x = state[0];
        float y = state[1];

        if (x < 0 || x >= grid_size || y < 0 || y >= grid_size) {
            particles.weight[i] = 0.0f; // Particle is out of bounds
        }
        
        int pos_x = clamp(static_cast<int>(x), 0, grid_size - 1);
        int pos_y = clamp(static_cast<int>(y), 0, grid_size - 1);
        particles.grid_cell_idx[i] = pos_y * grid_size + pos_x;
    }
}

} // namespace kernel
} // namespace dogm

#include "dogm/kernel/init.h"
#include <numeric>

namespace dogm {
namespace kernel {

void initGridCells(std::vector<GridCell>& grid_cells, std::vector<MeasurementCell>& meas_cells) {
    for (auto& cell : grid_cells) {
        cell = GridCell();
    }
    for (auto& cell : meas_cells) {
        cell = MeasurementCell();
    }
}

void initParticles(ParticlesSoA& particles, RandomGenerator& rng, float max_velocity, int grid_size) {
    float new_weight = 1.0f / particles.size();

    #pragma omp parallel for
    for (size_t i = 0; i < particles.size(); ++i) {
        float x = rng.uniform(0.0f, grid_size - 1.0f);
        float y = rng.uniform(0.0f, grid_size - 1.0f);
        float vx = rng.uniform(-max_velocity, max_velocity);
        float vy = rng.uniform(-max_velocity, max_velocity);
        
        particles.state[i] = Vec4(x, y, vx, vy);
        particles.weight[i] = new_weight;
        particles.grid_cell_idx[i] = static_cast<int>(y) * grid_size + static_cast<int>(x);
        particles.associated[i] = false;
    }
}

void initNewParticles(ParticlesSoA& birth_particles, const std::vector<GridCell>& grid_cells,
                      const std::vector<MeasurementCell>& meas_cells,
                      const std::vector<float>& born_masses_array, RandomGenerator& rng,
                      const DOGM::Params& params, int grid_size) {

    std::vector<float> particle_orders_accum;
    accumulate(born_masses_array, particle_orders_accum);

    if (particle_orders_accum.empty() || particle_orders_accum.back() == 0) {
        #pragma omp parallel for
        for (size_t i = 0; i < birth_particles.size(); ++i) {
             birth_particles.weight[i] = 0.0f;
        }
        return;
    }
    
    float total_born_mass = particle_orders_accum.back();
    float v_B = birth_particles.size();
    
    #pragma omp parallel for
    for (int j = 0; j < grid_cells.size(); ++j) {
        float start_order = (j == 0) ? 0.0f : particle_orders_accum[j-1];
        float end_order = particle_orders_accum[j];
        
        int start_idx = static_cast<int>(std::ceil(start_order / total_born_mass * v_B));
        int end_idx = static_cast<int>(std::ceil(end_order / total_born_mass * v_B));

        const auto& meas_cell = meas_cells[j];
        float p_A = meas_cell.p_A;

        int num_new_particles = end_idx - start_idx;
        if (num_new_particles <= 0) continue;

        int nu_A = static_cast<int>(roundf(num_new_particles * p_A));
        int nu_UA = num_new_particles - nu_A;

        float w_A = (nu_A > 0) ? (p_A * born_masses_array[j]) / nu_A : 0.0f;
        float w_UA = (nu_UA > 0) ? ((1.0f - p_A) * born_masses_array[j]) / nu_UA : 0.0f;

        for (int i = start_idx; i < end_idx; ++i) {
            float grid_x = j % grid_size + 0.5f;
            float grid_y = j / grid_size + 0.5f;
            
            bool is_associated = (i < start_idx + nu_A);
            
            float vx, vy;
            if (is_associated && meas_cell.velocity_confidence > 0.5f) {
                // Sample around measured radial velocity
                float cell_angle = atan2(grid_y - 1.5f, grid_x - 1.5f);
                float mean_vx = meas_cell.radial_velocity * cos(cell_angle);
                float mean_vy = meas_cell.radial_velocity * sin(cell_angle);
                vx = rng.normal(mean_vx, params.stddev_velocity / 2.0f);
                vy = rng.normal(mean_vy, params.stddev_velocity / 2.0f);
            } else {
                vx = rng.normal(0.0f, params.stddev_velocity);
                vy = rng.normal(0.0f, params.stddev_velocity);
            }

            birth_particles.state[i] = Vec4(grid_x, grid_y, vx, vy);
            birth_particles.weight[i] = is_associated ? w_A : w_UA;
            birth_particles.grid_cell_idx[i] = j;
            birth_particles.associated[i] = is_associated;
        }
    }
}

} // namespace kernel
} // namespace dogm

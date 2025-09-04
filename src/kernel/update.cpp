#include "dogm/kernel/update.h"
#include <algorithm>
#include <vector>

namespace dogm {
namespace kernel {

void particleToGrid(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                    std::vector<float>& weight_array) {

    // This part is tricky to parallelize efficiently without sorting.
    // A simpler approach for CPU is to iterate and mark start/end.
    // For performance, we can pre-sort particles by cell index.
    
    // Create a vector of indices to sort
    std::vector<int> p_indices(particles.size());
    std::iota(p_indices.begin(), p_indices.end(), 0);

    // Sort indices based on particle's grid_cell_idx
    std::sort(p_indices.begin(), p_indices.end(),
        [&](int a, int b) {
            return particles.grid_cell_idx[a] < particles.grid_cell_idx[b];
        });

    // Create sorted temporary particle storage
    ParticlesSoA sorted_particles;
    sorted_particles.resize(particles.size());
    #pragma omp parallel for
    for(size_t i = 0; i < particles.size(); ++i) {
        sorted_particles.state[i] = particles.state[p_indices[i]];
        sorted_particles.grid_cell_idx[i] = particles.grid_cell_idx[p_indices[i]];
        sorted_particles.weight[i] = particles.weight[p_indices[i]];
        sorted_particles.associated[i] = particles.associated[p_indices[i]];
    }
    // Now use the sorted particles
    const_cast<ParticlesSoA&>(particles) = sorted_particles;
    
    #pragma omp parallel for
    for(size_t i = 0; i < grid_cells.size(); ++i) {
        grid_cells[i].start_idx = -1;
        grid_cells[i].end_idx = -1;
    }

    if (particles.size() == 0) return;

    for (size_t i = 0; i < particles.size(); ++i) {
        int cell_idx = particles.grid_cell_idx[i];
        if (grid_cells[cell_idx].start_idx == -1) {
            grid_cells[cell_idx].start_idx = i;
        }
        grid_cells[cell_idx].end_idx = i;
        weight_array[i] = particles.weight[i];
    }
}

void updateOccupancy(std::vector<GridCell>& grid_cells, const ParticlesSoA& particles,
                     const std::vector<float>& weight_array,
                     const std::vector<MeasurementCell>& meas_cells,
                     std::vector<float>& born_masses_array,
                     const DOGM::Params& params, float dt) {

    std::vector<float> weight_accum;
    accumulate(weight_array, weight_accum);

    #pragma omp parallel for
    for (size_t i = 0; i < grid_cells.size(); ++i) {
        auto& cell = grid_cells[i];
        const auto& meas_cell = meas_cells[i];

        float m_occ_pred = 0.0f;
        if (cell.start_idx != -1) {
            m_occ_pred = subtract(weight_accum, cell.start_idx, cell.end_idx);
        }
        
        m_occ_pred = clamp(m_occ_pred, 0.0f, 1.0f);
        
        float freespace_discount_factor = std::pow(params.freespace_discount, dt);
        float m_free_pred = std::min(freespace_discount_factor * cell.free_mass, 1.0f - m_occ_pred);

        // Dempster-Shafer combination
        float unknown_pred = 1.0f - m_occ_pred - m_free_pred;
        float meas_unknown = 1.0f - meas_cell.free_mass - meas_cell.occ_mass;
        float K = m_free_pred * meas_cell.occ_mass + m_occ_pred * meas_cell.free_mass;

        float m_occ_up = (m_occ_pred * meas_unknown + unknown_pred * meas_cell.occ_mass + m_occ_pred * meas_cell.occ_mass) / (1.0f - K);
        float m_free_up = (m_free_pred * meas_unknown + unknown_pred * meas_cell.free_mass + m_free_pred * meas_cell.free_mass) / (1.0f - K);

        float rho_b = (m_occ_up * params.birth_prob * (1.0f - m_occ_pred)) / (m_occ_pred + params.birth_prob * (1.0f - m_occ_pred) + 1e-9);
        float rho_p = m_occ_up - rho_b;
        
        born_masses_array[i] = clamp(rho_b, 0.0f, 1.0f);
        
        cell.pers_occ_mass = clamp(rho_p, 0.0f, 1.0f);
        cell.new_born_occ_mass = clamp(rho_b, 0.0f, 1.0f);
        cell.free_mass = clamp(m_free_up, 0.0f, 1.0f);
        cell.occ_mass = clamp(m_occ_up, 0.0f, 1.0f);
        cell.pred_occ_mass = m_occ_pred;
    }
}

void updatePersistent(ParticlesSoA& particles, const std::vector<MeasurementCell>& meas_cells,
                      std::vector<GridCell>& grid_cells, std::vector<float>& weight_array) {
    
    // Kernel 1: Update unnormalized weights
    #pragma omp parallel for
    for(size_t i = 0; i < particles.size(); ++i) {
        int cell_idx = particles.grid_cell_idx[i];
        weight_array[i] = meas_cells[cell_idx].likelihood * particles.weight[i];
    }
    
    std::vector<float> weight_accum;
    accumulate(weight_array, weight_accum);

    // Kernel 2: Calculate normalization components
    #pragma omp parallel for
    for(size_t i = 0; i < grid_cells.size(); ++i) {
        auto& cell = grid_cells[i];
        if(cell.start_idx != -1) {
            float m_occ_accum = subtract(weight_accum, cell.start_idx, cell.end_idx);
            cell.mu_A = (m_occ_accum > 0.0f) ? cell.pers_occ_mass / m_occ_accum : 0.0f;
            cell.mu_UA = (cell.pred_occ_mass > 0.0f) ? cell.pers_occ_mass / cell.pred_occ_mass : 0.0f;
        } else {
            cell.mu_A = 0.0f;
            cell.mu_UA = 0.0f;
        }
    }
    
    // Kernel 3: Normalize weights
    #pragma omp parallel for
    for(size_t i = 0; i < particles.size(); ++i) {
        int cell_idx = particles.grid_cell_idx[i];
        const auto& cell = grid_cells[cell_idx];
        const auto& meas_cell = meas_cells[cell_idx];
        
        float new_weight = meas_cell.p_A * cell.mu_A * weight_array[i] + (1.0f - meas_cell.p_A) * cell.mu_UA * particles.weight[i];
        
        // Add velocity likelihood for radar fusion
        if(meas_cell.velocity_confidence > 0.5f) {
            float dx = particles.state[i][0] - 1.5f;
            float dy = particles.state[i][1] - 1.5f;
            float angle = std::atan2(dy, dx);
            float particle_radial_vel = particles.state[i][2] * cos(angle) + particles.state[i][3] * sin(angle);
            float vel_diff = particle_radial_vel - meas_cell.radial_velocity;
            float stddev = 0.5f; // Velocity measurement noise
            float vel_likelihood = exp(-0.5f * vel_diff * vel_diff / (stddev * stddev));
            new_weight *= vel_likelihood;
        }
        
        weight_array[i] = new_weight;
    }
}

void computeStatisticalMoments(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                               const std::vector<float>& weight_array) {
    
    #pragma omp parallel for
    for (size_t i = 0; i < grid_cells.size(); ++i) {
        auto& cell = grid_cells[i];
        
        if (cell.start_idx == -1 || cell.pers_occ_mass == 0.0f) {
            cell.mean_x_vel = cell.mean_y_vel = 0.0f;
            cell.var_x_vel = cell.var_y_vel = cell.covar_xy_vel = 0.0f;
            continue;
        }
        
        float sum_vx = 0.0f, sum_vy = 0.0f;
        float sum_vx2 = 0.0f, sum_vy2 = 0.0f, sum_vxy = 0.0f;
        
        for (int p_idx = cell.start_idx; p_idx <= cell.end_idx; ++p_idx) {
            float w = weight_array[p_idx];
            float vx = particles.state[p_idx][2];
            float vy = particles.state[p_idx][3];
            
            sum_vx += w * vx;
            sum_vy += w * vy;
            sum_vx2 += w * vx * vx;
            sum_vy2 += w * vy * vy;
            sum_vxy += w * vx * vy;
        }
        
        float inv_rho = 1.0f / cell.pers_occ_mass;
        float mean_x = inv_rho * sum_vx;
        float mean_y = inv_rho * sum_vy;
        
        cell.mean_x_vel = mean_x;
        cell.mean_y_vel = mean_y;
        cell.var_x_vel = inv_rho * sum_vx2 - mean_x * mean_x;
        cell.var_y_vel = inv_rho * sum_vy2 - mean_y * mean_y;
        cell.covar_xy_vel = inv_rho * sum_vxy - mean_x * mean_y;
    }
}

} // namespace kernel
} // namespace dogm

#include "dogm/kernel/update.h"
#include <algorithm>
#include <vector>
#include "dogm/common.h" // accumulate, subtract를 위해 추가

namespace dogm {
namespace kernel {

void particleToGrid(const ParticlesSoA& particles, std::vector<GridCell>& grid_cells,
                    std::vector<float>& weight_array) {

    // ... (이하 기존 코드와 동일) ...
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
    }
    // Now use the sorted particles
    const_cast<ParticlesSoA&>(particles) = sorted_particles;
    
    #pragma omp parallel for
    for(size_t i = 0; i < grid_cells.size(); ++i) {
        grid_cells[i].start_idx = -1;
        grid_cells[i].end_idx = -1;
    }

    if (particles.size() == 0) return;

    // This part now needs to be careful due to sorting
    grid_cells[particles.grid_cell_idx[0]].start_idx = 0;
    for (size_t i = 1; i < particles.size(); ++i) {
        weight_array[i-1] = particles.weight[i-1];
        int prev_cell_idx = particles.grid_cell_idx[i-1];
        int cell_idx = particles.grid_cell_idx[i];
        if (cell_idx != prev_cell_idx) {
            grid_cells[prev_cell_idx].end_idx = i - 1;
            grid_cells[cell_idx].start_idx = i;
        }
    }
    grid_cells[particles.grid_cell_idx.back()].end_idx = particles.size() - 1;
    weight_array.back() = particles.weight.back();
}


// 'const ParticlesSoA& particles' 인자 제거
void updateOccupancy(std::vector<GridCell>& grid_cells,
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

// 'const Vec2& ego_pose' 인자 추가
void updatePersistent(ParticlesSoA& particles, const std::vector<MeasurementCell>& meas_cells,
                      std::vector<GridCell>& grid_cells, std::vector<float>& weight_array,
                      const Vec2& ego_pose) {
    
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
            // [BUG FIX] 하드코딩된 값(1.5f) 대신 실제 로봇의 위치(ego_pose) 사용
            float dx = particles.state[i][0] - ego_pose.x();
            float dy = particles.state[i][1] - ego_pose.y();
            float angle = std::atan2(dy, dx);
            float particle_radial_vel = particles.state[i][2] * cos(angle) + particles.state[i][3] * sin(angle);
            float vel_diff = particle_radial_vel - meas_cell.radial_velocity;
            
            // SNR 기반으로 속도 측정 노이즈 조절 (신뢰도 높을수록 오차에 민감)
            float stddev = 0.5f * (1.0f - meas_cell.velocity_confidence * 0.8f);
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
        float total_weight = 0.0f; // 가중치 총합을 구하기 위한 변수
        
        for (int p_idx = cell.start_idx; p_idx <= cell.end_idx; ++p_idx) {
            float w = weight_array[p_idx];
            float vx = particles.state[p_idx][2];
            float vy = particles.state[p_idx][3];
            
            sum_vx += w * vx;
            sum_vy += w * vy;
            sum_vx2 += w * vx * vx;
            sum_vy2 += w * vy * vy;
            sum_vxy += w * vx * vy;
            total_weight += w;
        }
        
        if (total_weight < 1e-9) continue; // 가중치 합이 0에 가까우면 계산 생략

        float inv_rho = 1.0f / total_weight; // pers_occ_mass 대신 실제 가중치 합으로 정규화
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
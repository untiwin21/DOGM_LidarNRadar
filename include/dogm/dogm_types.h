#pragma once

#include <vector>
#include <Eigen/Dense>

namespace dogm {

using Vec2 = Eigen::Vector2f;
using Vec4 = Eigen::Vector4f;

struct GridCell {
    int start_idx = -1;
    int end_idx = -1;
    float new_born_occ_mass = 0.0f;
    float pers_occ_mass = 0.0f;
    float free_mass = 0.0f;
    float occ_mass = 0.0f;
    float pred_occ_mass = 0.0f;
    float mu_A = 0.0f;
    float mu_UA = 0.0f;
    float w_A = 0.0f;
    float w_UA = 0.0f;
    
    float mean_x_vel = 0.0f;
    float mean_y_vel = 0.0f;
    float var_x_vel = 0.0f;
    float var_y_vel = 0.0f;
    float covar_xy_vel = 0.0f;
};

struct MeasurementCell {
    float free_mass = 0.0f;
    float occ_mass = 0.0f;
    float likelihood = 1.0f;
    float p_A = 1.0f;
    
    // Radar specific
    float radial_velocity = 0.0f;
    float velocity_confidence = 0.0f;
};

struct Particle {
    Vec4 state;  // x, y, vx, vy
    int grid_cell_idx;
    float weight;
    bool associated;
    
    Particle() : state(0, 0, 0, 0), grid_cell_idx(0), weight(0), associated(false) {}
};

struct ParticlesSoA {
    std::vector<Vec4> state;
    std::vector<int> grid_cell_idx;
    std::vector<float> weight;
    std::vector<bool> associated;
    
    size_t size() const { return state.size(); }
    
    void resize(size_t new_size) {
        state.resize(new_size);
        grid_cell_idx.resize(new_size);
        weight.resize(new_size);
        associated.resize(new_size);
    }
    
    void clear() {
        state.clear();
        grid_cell_idx.clear();
        weight.clear();
        associated.clear();
    }
};

struct LidarMeasurement {
    std::vector<float> ranges;
    std::vector<float> angles;
    float max_range = 3.0f;  // 3m for indoor robot
};

struct RadarDetection {
    Vec2 position;
    float radial_velocity;
    float intensity;
};

struct RadarMeasurement {
    std::vector<RadarDetection> detections;
};

struct SensorFrame {
    double timestamp;
    LidarMeasurement lidar;
    RadarMeasurement radar;
    Vec2 ego_pose;
    float ego_yaw;
};

} // namespace dogm
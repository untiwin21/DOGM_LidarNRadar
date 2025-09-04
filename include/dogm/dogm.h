#pragma once

#include "dogm_types.h"
#include "common.h"
#include <memory>

namespace dogm {

class DOGM {
public:
    struct Params {
        float size = 3.0f;                    // Grid size [m]
        float resolution = 0.1f;               // 10cm resolution
        int particle_count = 10000;           // Reduced for CPU
        int new_born_particle_count = 1000;   
        float persistence_prob = 0.99f;
        float stddev_process_noise_position = 0.02f;
        float stddev_process_noise_velocity = 0.5f;
        float birth_prob = 0.02f;
        float stddev_velocity = 1.0f;         // 1 m/s for indoor
        float init_max_velocity = 3.0f;       // 3 m/s max
        float freespace_discount = 0.01f;
    };
    
    DOGM(const Params& params);
    ~DOGM();
    
    void updateGrid(const SensorFrame& frame, float dt);
    
    const std::vector<GridCell>& getGridCells() const { return grid_cells; }
    const std::vector<MeasurementCell>& getMeasurementCells() const { return meas_cells; }
    const ParticlesSoA& getParticles() const { return particles; }
    
    int getGridSize() const { return grid_size; }
    float getResolution() const { return params.resolution; }
    
private:
    void initialize();
    void updateMeasurementGrid(const SensorFrame& frame);
    void particlePrediction(float dt);
    void particleAssignment();
    void gridCellOccupancyUpdate(float dt);
    void updatePersistentParticles();
    void initializeNewParticles();
    void statisticalMoments();
    void resampling();
    
    Params params;
    int grid_size;
    int grid_cell_count;
    
    std::vector<GridCell> grid_cells;
    std::vector<MeasurementCell> meas_cells;
    
    ParticlesSoA particles;
    ParticlesSoA particles_next;
    ParticlesSoA birth_particles;
    
    std::vector<float> weight_array;
    std::vector<float> birth_weight_array;
    std::vector<float> born_masses_array;
    
    std::unique_ptr<RandomGenerator> rng;
    
    bool first_update = true;
    Vec2 ego_pose;
    float ego_yaw = 0.0f;
};

} // namespace dogm

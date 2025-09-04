#include "dogm/dogm.h"
#include "dogm/kernel/init.h"
#include "dogm/kernel/predict.h"
#include "dogm/kernel/update.h"
#include "dogm/kernel/resampling.h"
#include "dogm/kernel/sensor_fusion.h"
#include <algorithm>
#include <numeric>

namespace dogm {

DOGM::DOGM(const Params& params) 
    : params(params),
      grid_size(static_cast<int>(params.size / params.resolution)),
      grid_cell_count(grid_size * grid_size),
      rng(std::make_unique<RandomGenerator>()) {
    
    initialize();
}

DOGM::~DOGM() = default;

void DOGM::initialize() {
    // Allocate memory
    grid_cells.resize(grid_cell_count);
    meas_cells.resize(grid_cell_count);
    
    particles.resize(params.particle_count);
    particles_next.resize(params.particle_count);
    birth_particles.resize(params.new_born_particle_count);
    
    weight_array.resize(params.particle_count);
    birth_weight_array.resize(params.new_born_particle_count);
    born_masses_array.resize(grid_cell_count);
    
    // Initialize particles
    kernel::initParticles(particles, *rng, params.init_max_velocity, grid_size);
    kernel::initGridCells(grid_cells, meas_cells);
}

void DOGM::updateGrid(const SensorFrame& frame, float dt) {
    // Update measurement grid with sensor fusion
    updateMeasurementGrid(frame);
    
    // Update ego motion
    if (!first_update) {
        Vec2 delta = frame.ego_pose - ego_pose;
        // Apply ego motion compensation if needed
    }
    ego_pose = frame.ego_pose;
    ego_yaw = frame.ego_yaw;
    first_update = false;
    
    // Main DOGM pipeline
    particlePrediction(dt);
    particleAssignment();
    gridCellOccupancyUpdate(dt);
    updatePersistentParticles();
    initializeNewParticles();
    statisticalMoments();
    resampling();
    
    // Swap particle buffers
    std::swap(particles, particles_next);
}

void DOGM::updateMeasurementGrid(const SensorFrame& frame) {
    // Fuse LiDAR and Radar data
    kernel::fuseAndCreateMeasurementGrid(
        meas_cells, 
        frame.lidar, 
        frame.radar,
        grid_size, 
        params.resolution
    );
}

void DOGM::particlePrediction(float dt) {
    kernel::predict(particles, *rng, params, grid_size, dt);
}

void DOGM::particleAssignment() {
    kernel::particleToGrid(particles, grid_cells, weight_array);
}

void DOGM::gridCellOccupancyUpdate(float dt) {
    kernel::updateOccupancy(grid_cells, particles, weight_array, 
                           meas_cells, born_masses_array, params, dt);
}

void DOGM::updatePersistentParticles() {
    kernel::updatePersistent(particles, meas_cells, grid_cells, weight_array);
}

void DOGM::initializeNewParticles() {
    kernel::initNewParticles(birth_particles, grid_cells, meas_cells,
                            born_masses_array, *rng, params, grid_size);
}

void DOGM::statisticalMoments() {
    kernel::computeStatisticalMoments(particles, grid_cells, weight_array);
}

void DOGM::resampling() {
    kernel::resample(particles, particles_next, birth_particles,
                    weight_array, birth_weight_array, *rng, params);
}

} // namespace dogm

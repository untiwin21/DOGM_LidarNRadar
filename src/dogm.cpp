#include "dogm/dogm.h"
#include "dogm/kernel/init.h"
#include "dogm/kernel/predict.h"
#include "dogm/kernel/update.h"
#include "dogm/kernel/resampling.h"
#include "dogm/kernel/sensor_fusion.h" // sensor_fusion.h 헤더를 포함합니다.
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
    grid_cells.resize(grid_cell_count);
    meas_cells.resize(grid_cell_count);
    
    particles.resize(params.particle_count);
    particles_next.resize(params.particle_count);
    birth_particles.resize(params.new_born_particle_count);
    
    weight_array.resize(params.particle_count);
    birth_weight_array.resize(params.new_born_particle_count);
    born_masses_array.resize(grid_cell_count);
    
    kernel::initGridCells(grid_cells);
    kernel::initParticles(particles, *rng, params.init_max_velocity, grid_size, params.resolution);
}

void DOGM::updateGrid(const SensorFrame& frame, float dt) {
    // 프레임에서 ego_pose와 ego_yaw를 클래스 멤버 변수로 업데이트
    this->ego_pose = frame.ego_pose;
    this->ego_yaw = frame.ego_yaw;

    updateMeasurementGrid(frame);
    
    // TODO: Implement ego motion compensation based on frame.ego_pose
    
    particlePrediction(dt);
    particleAssignment();
    // gridCellOccupancyUpdate의 인자에서 particles 제거
    gridCellOccupancyUpdate(dt);
    updatePersistentParticles();
    initializeNewParticles();
    statisticalMoments();
    resampling();
    
    std::swap(particles, particles_next);
}

void DOGM::updateMeasurementGrid(const SensorFrame& frame) {
    // fuseAndCreateMeasurementGrid 함수를 호출하도록 변경합니다.
    kernel::fuseAndCreateMeasurementGrid(meas_cells, frame, grid_size, params.resolution, ego_pose, ego_yaw);
}

// 나머지 함수들은 기존과 동일합니다.
void DOGM::particlePrediction(float dt) {
    kernel::predict(particles, *rng, params, grid_size, params.resolution, dt);
}

void DOGM::particleAssignment() {
    kernel::particleToGrid(particles, grid_cells, weight_array);
}

void DOGM::gridCellOccupancyUpdate(float dt) {
    // updateOccupancy 함수 시그니처 변경에 따라 particles 인자 제거
    kernel::updateOccupancy(grid_cells, weight_array, meas_cells, born_masses_array, params, dt);
}

void DOGM::updatePersistentParticles() {
    // ego_pose를 넘겨주도록 수정
    kernel::updatePersistent(particles, meas_cells, grid_cells, weight_array, ego_pose);
}

void DOGM::initializeNewParticles() {
    kernel::initNewParticles(birth_particles, grid_cells, meas_cells, born_masses_array, *rng, params, grid_size, params.resolution);
}

void DOGM::statisticalMoments() {
    kernel::computeStatisticalMoments(particles, grid_cells, weight_array);
}

void DOGM::resampling() {
    kernel::resample(particles, particles_next, birth_particles, weight_array, birth_weight_array, *rng);
}

} // namespace dogm
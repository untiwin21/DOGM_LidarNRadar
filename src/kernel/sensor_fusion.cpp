#include "dogm/kernel/sensor_fusion.h"
#include <cmath>

namespace dogm {
namespace kernel {

static float inverseSensorModelLidar(float range, float measured_range, 
                                    float max_range, float resolution) {
    const float stddev = 0.05f;  // 5cm stddev for LiDAR
    
    if (!std::isfinite(measured_range)) {
        // No return - high probability of free space
        if (range < max_range * 0.9f) {
            return 0.1f;  // Low occupancy probability
        }
        return 0.5f;  // Unknown
    }
    
    float diff = std::abs(range - measured_range);
    
    if (range < measured_range - 2 * stddev) {
        // Before obstacle - free space
        return 0.2f;
    } else if (diff < 2 * stddev) {
        // At obstacle - occupied
        float occ_prob = 0.9f * std::exp(-0.5f * diff * diff / (stddev * stddev));
        return occ_prob;
    } else {
        // Behind obstacle - unknown
        return 0.5f;
    }
}

void fuseAndCreateMeasurementGrid(std::vector<MeasurementCell>& meas_cells,
                                 const LidarMeasurement& lidar,
                                 const RadarMeasurement& radar,
                                 int grid_size, float resolution) {
    
    // Clear measurement grid
    std::fill(meas_cells.begin(), meas_cells.end(), MeasurementCell());
    
    // Process LiDAR measurements
    #pragma omp parallel for
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int idx = y * grid_size + x;
            
            // Convert grid coordinates to world coordinates
            float world_x = x * resolution;
            float world_y = y * resolution;
            
            // Find nearest LiDAR measurement
            float min_occ_prob = 1.0f;
            float max_free_prob = 0.0f;
            
            for (size_t i = 0; i < lidar.ranges.size(); ++i) {
                if (i >= lidar.angles.size()) continue;
                
                float angle = lidar.angles[i];
                float measured_range = lidar.ranges[i];
                
                // Calculate range from sensor to grid cell
                float dx = world_x - 1.5f;  // Sensor at center (1.5m)
                float dy = world_y - 1.5f;
                float range = std::sqrt(dx*dx + dy*dy);
                float cell_angle = std::atan2(dy, dx);
                
                // Check if cell is within this laser beam (simple approximation)
                float angle_diff = std::abs(cell_angle - angle);
                if (angle_diff > 0.01f) continue;  // ~0.5 degree tolerance
                
                float occ_prob = inverseSensorModelLidar(range, measured_range, 
                                                        lidar.max_range, resolution);
                
                if (occ_prob > 0.5f) {
                    min_occ_prob = std::min(min_occ_prob, occ_prob);
                } else {
                    max_free_prob = std::max(max_free_prob, 1.0f - occ_prob);
                }
            }
            
            // Set occupancy masses based on LiDAR
            meas_cells[idx].occ_mass = (min_occ_prob < 1.0f) ? min_occ_prob : 0.5f;
            meas_cells[idx].free_mass = max_free_prob;
            
            // Ensure masses don't exceed 1
            float total = meas_cells[idx].occ_mass + meas_cells[idx].free_mass;
            if (total > 1.0f) {
                meas_cells[idx].occ_mass /= total;
                meas_cells[idx].free_mass /= total;
            }
        }
    }
    
    // Process Radar measurements - update velocity information
    for (const auto& detection : radar.detections) {
        // Convert position to grid coordinates
        int grid_x = static_cast<int>(detection.position[0] / resolution);
        int grid_y = static_cast<int>(detection.position[1] / resolution);
        
        if (grid_x < 0 || grid_x >= grid_size || 
            grid_y < 0 || grid_y >= grid_size) continue;
        
        int idx = grid_y * grid_size + grid_x;
        
        // Update velocity information
        meas_cells[idx].radial_velocity = detection.radial_velocity;
        meas_cells[idx].velocity_confidence = 0.8f;  // High confidence for radar velocity
        
        // Radar also indicates occupancy
        meas_cells[idx].occ_mass = std::max(meas_cells[idx].occ_mass, 0.7f);
        
        // Update surrounding cells with lower confidence
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                
                int nx = grid_x + dx;
                int ny = grid_y + dy;
                
                if (nx >= 0 && nx < grid_size && ny >= 0 && ny < grid_size) {
                    int nidx = ny * grid_size + nx;
                    meas_cells[nidx].radial_velocity = detection.radial_velocity;
                    meas_cells[nidx].velocity_confidence = 0.4f;  // Lower confidence
                }
            }
        }
    }
    
    // Set likelihood and p_A for all cells
    #pragma omp parallel for
    for (int i = 0; i < grid_size * grid_size; ++i) {
        meas_cells[i].likelihood = 1.0f;
        meas_cells[i].p_A = meas_cells[i].velocity_confidence > 0 ? 0.9f : 0.5f;
    }
}

} // namespace kernel
} // namespace dogm

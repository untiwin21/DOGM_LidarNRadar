#include "dogm/kernel/sensor_fusion.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace dogm {
namespace kernel {

// SNR 값을 0.0 ~ 1.0 범위의 신뢰도로 변환하는 함수
// SNR이 5 미만이면 신뢰도 0, 20 이상이면 신뢰도 1로 계산합니다.
float snrToConfidence(float snr, float min_snr = 5.0f, float max_snr = 20.0f) {
    if (snr < min_snr) return 0.0f;
    if (snr > max_snr) return 1.0f;
    return (snr - min_snr) / (max_snr - min_snr);
}

void fuseAndCreateMeasurementGrid(std::vector<MeasurementCell>& meas_cells,
                                 const SensorFrame& frame,
                                 int grid_size, float resolution,
                                 const Vec2& ego_pose, float ego_yaw) {
    
    // 1. 측정 그리드를 비어있는 상태(Unknown)로 초기화
    std::fill(meas_cells.begin(), meas_cells.end(), MeasurementCell());

    // 2. Lidar 데이터 처리: Inverse Sensor Model 적용 (간략화된 버전)
    // 각 Lidar 측정치에 대해 광선을 따라가며 Free-space와 Occupied를 계산
    for (size_t i = 0; i < frame.lidar.ranges.size(); ++i) {
        float angle = frame.lidar.angles[i];
        float range = frame.lidar.ranges[i];
        
        // 로봇 좌표계 기준 측정된 끝점
        float end_x = range * std::cos(angle);
        float end_y = range * std::sin(angle);

        // 광선을 따라 그리드 셀을 업데이트
        for (float r = 0; r < range; r += resolution) {
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);
            int grid_x = static_cast<int>((ego_pose.x() + x) / resolution);
            int grid_y = static_cast<int>((ego_pose.y() + y) / resolution);

            if (grid_x < 0 || grid_x >= grid_size || grid_y < 0 || grid_y >= grid_size) continue;
            int idx = grid_y * grid_size + grid_x;
            
            // 다른 측정에 의해 점유된 셀은 덮어쓰지 않고, 비어있을 확률만 높임
            meas_cells[idx].free_mass = std::max(meas_cells[idx].free_mass, 0.7f);
            meas_cells[idx].occ_mass *= 0.5f; // free일 확률이 높으므로 occ 확률은 감소
        }
        
        // 끝점은 점유 상태로 업데이트
        int grid_x = static_cast<int>((ego_pose.x() + end_x) / resolution);
        int grid_y = static_cast<int>((ego_pose.y() + end_y) / resolution);
        if (grid_x < 0 || grid_x >= grid_size || grid_y < 0 || grid_y >= grid_size) continue;
        int idx = grid_y * grid_size + grid_x;
        meas_cells[idx].occ_mass = std::max(meas_cells[idx].occ_mass, 0.8f);
        meas_cells[idx].free_mass = 0.0f; // 점유되었으므로 free일 확률은 0
    }

    // 3. Radar 데이터 처리 및 퓨전
    for (const auto& detection : frame.radar) {
        int grid_x = static_cast<int>(detection.position.x() / resolution);
        int grid_y = static_cast<int>(detection.position.y() / resolution);

        if (grid_x < 0 || grid_x >= grid_size || grid_y < 0 || grid_y >= grid_size) continue;
        
        int idx = grid_y * grid_size + grid_x;

        // SNR을 이용해 점유 확률과 속도 신뢰도를 계산
        float confidence = snrToConfidence(detection.snr);
        
        // Radar 탐지 지점은 점유 확률을 높임 (기존 Lidar 정보와 max 연산)
        meas_cells[idx].occ_mass = std::max(meas_cells[idx].occ_mass, 0.7f * confidence);
        meas_cells[idx].free_mass *= (1.0f - confidence); // Radar 탐지 지점의 free_mass 감소
        
        // 속도 정보 업데이트 (더 높은 신뢰도의 측정값으로 갱신)
        if (confidence > meas_cells[idx].velocity_confidence) {
             meas_cells[idx].radial_velocity = detection.radial_velocity;
             meas_cells[idx].velocity_confidence = confidence;
        }
    }
    
    // 4. 모든 셀에 대해 확률 정규화 및 likelihood, p_A 설정
    #pragma omp parallel for
    for (size_t i = 0; i < meas_cells.size(); ++i) {
        float total_mass = meas_cells[i].occ_mass + meas_cells[i].free_mass;
        if (total_mass > 1.0f) { // 확률의 합이 1을 넘지 않도록 정규화
            meas_cells[i].occ_mass /= total_mass;
            meas_cells[i].free_mass /= total_mass;
        }
        // likelihood는 기본값 1.0 사용
        meas_cells[i].likelihood = 1.0f; 
        // 속도 신뢰도가 높을수록 연관 확률(p_A)을 높게 설정
        meas_cells[i].p_A = 0.5f + 0.4f * meas_cells[i].velocity_confidence; 
    }
}

} // namespace kernel
} // namespace dogm
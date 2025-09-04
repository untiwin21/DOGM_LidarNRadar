#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <Eigen/Dense>

struct VisGridCell {
    float occ_prob = 0.5f;
    Eigen::Vector2f mean_vel = {0.0f, 0.0f};
};

using GridState = std::vector<VisGridCell>;

namespace dogm {
    cv::Mat visualizeDOGM(const GridState& grid_state, int grid_size);
    void addInfoText(cv::Mat& image, double timestamp);
}
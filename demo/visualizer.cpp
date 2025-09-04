#include "visualizer.h"

namespace dogm {

cv::Mat visualizeDOGM(const GridState& grid_state, int grid_size) {
    int vis_scale = 8;
    cv::Mat img(grid_size * vis_scale, grid_size * vis_scale, CV_8UC3, cv::Scalar(128, 128, 128));

    #pragma omp parallel for
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            const auto& cell = grid_state[y * grid_size + x];
            int intensity = static_cast<int>(255 * (1.0f - cell.occ_prob));
            cv::Scalar color(intensity, intensity, intensity);
            
            if (cell.mean_vel.norm() > 0.1f) {
                float angle = std::atan2(cell.mean_vel.y(), cell.mean_vel.x());
                int hue = static_cast<int>(((angle + M_PI) / (2.0 * M_PI)) * 180.0);
                cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
                cv::Mat bgr;
                cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
                color = cv::Scalar(bgr.at<cv::Vec3b>(0, 0));
            }
            
            cv::rectangle(img, 
                         cv::Point(x * vis_scale, y * vis_scale),
                         cv::Point((x + 1) * vis_scale, (y + 1) * vis_scale),
                         color, -1);
        }
    }
    return img;
}

void addInfoText(cv::Mat& image, double timestamp) {
    std::string time_text = "Timestamp: " + std::to_string(timestamp).substr(0, 5) + " s";
    cv::putText(image, time_text, cv::Point(10, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

} // namespace dogm
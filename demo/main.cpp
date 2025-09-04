#include <iostream>
#include <opencv2/opencv.hpp>
#include "dogm/dogm.h"
#include "data_loader.h"

using namespace dogm;

cv::Mat visualizeDOGM(const DOGM& dogm) {
    int grid_size = dogm.getGridSize();
    cv::Mat img(grid_size * 10, grid_size * 10, CV_8UC3, cv::Scalar(128, 128, 128));
    
    const auto& cells = dogm.getGridCells();
    const auto& particles = dogm.getParticles();
    
    // Draw grid cells
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int idx = y * grid_size + x;
            const auto& cell = cells[idx];
            
            // Calculate occupancy probability
            float occ_prob = cell.occ_mass / (cell.occ_mass + cell.free_mass + 0.001f);
            
            // Color based on occupancy
            int intensity = static_cast<int>(255 * (1.0f - occ_prob));
            cv::Scalar color(intensity, intensity, intensity);
            
            // Add velocity visualization if present
            if (std::abs(cell.mean_x_vel) > 0.1f || std::abs(cell.mean_y_vel) > 0.1f) {
                float angle = std::atan2(cell.mean_y_vel, cell.mean_x_vel);
                // Map angle to hue
                int hue = static_cast<int>((angle + M_PI) * 180.0 / M_PI / 2);
                cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
                cv::Mat rgb;
                cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
                color = cv::Scalar(rgb.at<cv::Vec3b>(0, 0));
            }
            
            cv::rectangle(img, 
                         cv::Point(x * 10, y * 10),
                         cv::Point((x + 1) * 10, (y + 1) * 10),
                         color, -1);
        }
    }
    
    // Draw particles (optional, might be slow)
    for (size_t i = 0; i < std::min(size_t(1000), particles.size()); ++i) {
        const auto& state = particles.state[i];
        int px = static_cast<int>(state[0] * 10);
        int py = static_cast<int>(state[1] * 10);
        
        if (px >= 0 && px < img.cols && py >= 0 && py < img.rows) {
            cv::circle(img, cv::Point(px, py), 1, cv::Scalar(0, 255, 0), -1);
        }
    }
    
    return img;
}

int main(int argc, char** argv) {
    // Initialize DOGM
    DOGM::Params params;
    params.size = 3.0f;               // 3m x 3m grid
    params.resolution = 0.1f;          // 10cm cells
    params.particle_count = 5000;     // Reduced for CPU performance
    params.new_born_particle_count = 500;
    params.persistence_prob = 0.99f;
    params.stddev_process_noise_position = 0.02f;
    params.stddev_process_noise_velocity = 0.2f;
    params.birth_prob = 0.02f;
    params.stddev_velocity = 1.0f;    // 1 m/s for indoor robots
    params.init_max_velocity = 3.0f;  // 3 m/s max
    
    DOGM dogm(params);
    
    // Load sensor data
    std::string data_path = (argc > 1) ? argv[1] : "./data/sample/";
    DataLoader loader(data_path);
    
    cv::namedWindow("DOGM Visualization", cv::WINDOW_NORMAL);
    
    // Process frames
    float dt = 0.1f;  // 10 Hz
    while (loader.hasNextFrame()) {
        SensorFrame frame = loader.getNextFrame();
        
        // Update DOGM
        auto start = std::chrono::high_resolution_clock::now();
        dogm.updateGrid(frame, dt);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "DOGM update took: " << duration.count() << " ms" << std::endl;
        
        // Visualize
        cv::Mat visualization = visualizeDOGM(dogm);
        
        // Add text info
        cv::putText(visualization, 
                   "Frame: " + std::to_string(loader.getCurrentFrameIndex()),
                   cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(255, 255, 255), 1);
        
        cv::imshow("DOGM Visualization", visualization);
        
        char key = cv::waitKey(30);  // ~30 FPS visualization
        if (key == 27) break;  // ESC to exit
        if (key == 32) cv::waitKey(0);  // Space to pause
    }
    
    cv::destroyAllWindows();
    return 0;
}

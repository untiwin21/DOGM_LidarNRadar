#include "visualizer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>

// CSV 파싱을 위한 구조체와 타입 정의
using TimedGridData = std::map<double, GridState>;

TimedGridData parse_output_file(const std::string& filepath, int grid_size) {
    std::ifstream file(filepath);
    if (!file.is_open()) throw std::runtime_error("Error: Cannot open file " + filepath);
    
    TimedGridData all_data;
    std::string line;
    std::getline(file, line); // 헤더 스킵

    while(std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        double timestamp;
        int x, y;
        VisGridCell cell;

        std::getline(ss, token, ','); timestamp = std::stod(token);
        std::getline(ss, token, ','); x = std::stoi(token);
        std::getline(ss, token, ','); y = std::stoi(token);
        std::getline(ss, token, ','); cell.occ_prob = std::stof(token);
        std::getline(ss, token, ','); cell.mean_vel.x() = std::stof(token);
        std::getline(ss, token, ','); cell.mean_vel.y() = std::stof(token);

        if (all_data.find(timestamp) == all_data.end()) {
            all_data[timestamp].resize(grid_size * grid_size);
        }
        all_data[timestamp][y * grid_size + x] = cell;
    }
    return all_data;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <output_dogm.csv> <grid_size_in_cells> <mode> [output_video.mp4]" << std::endl;
        std::cerr << "Modes: --animate OR --view" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    int grid_size = std::stoi(argv[2]);
    std::string mode = argv[3];
    
    try {
        TimedGridData data = parse_output_file(input_path, grid_size);

        if (mode == "--animate") {
            if (argc != 5) {
                 std::cerr << "Usage for animate: " << argv[0] << " <input.csv> <grid_size> --animate <output.mp4>" << std::endl;
                 return 1;
            }
            std::string output_video_path = argv[4];
            cv::VideoWriter video_writer;
            
            auto const& [first_timestamp, first_grid_state] = *data.begin();
            cv::Mat first_frame = dogm::visualizeDOGM(first_grid_state, grid_size);
            video_writer.open(output_video_path, cv::VideoWriter::fourcc('m','p','4','v'), 10, first_frame.size(), true);
            if (!video_writer.isOpened()) throw std::runtime_error("Could not open video writer");

            for(auto const& [timestamp, grid_state] : data) {
                cv::Mat frame = dogm::visualizeDOGM(grid_state, grid_size);
                dogm::addInfoText(frame, timestamp);
                video_writer.write(frame);
            }
            std::cout << "Animation saved to " << output_video_path << std::endl;

        } else if (mode == "--view") {
            cv::namedWindow("DOGM Visualization", cv::WINDOW_NORMAL);
            for(auto const& [timestamp, grid_state] : data) {
                cv::Mat frame = dogm::visualizeDOGM(grid_state, grid_size);
                dogm::addInfoText(frame, timestamp);
                cv::imshow("DOGM Visualization", frame);
                if (cv::waitKey(500) == 27) break; // ESC로 종료
            }
            cv::destroyAllWindows();
        } else {
             std::cerr << "Error: Unknown mode '" << mode << "'" << std::endl;
             return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
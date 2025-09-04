#include "dogm/dogm.h"
#include "data_loader.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

using namespace dogm;

float pignistic(const GridCell& cell) {
    return cell.occ_mass + 0.5f * (1.0f - cell.occ_mass - cell.free_mass);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_data.csv> <output_dogm.csv>" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];

    DOGM::Params params;
    params.size = 20.0f;
    params.resolution = 0.2f;
    params.particle_count = 10000;
    params.new_born_particle_count = 1000;
    params.init_max_velocity = 3.0f;
    
    DOGM dogm(params);
    RealDataLoader loader(input_path);
    std::ofstream output_file(output_path);

    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open output file " << output_path << std::endl;
        return 1;
    }

    output_file << "timestamp,cell_x,cell_y,occ_prob,mean_vx,mean_vy\n";

    double last_timestamp = -1.0;

    while(loader.hasNextFrame()) {
        SensorFrame frame = loader.getNextFrame();
        float dt = (last_timestamp < 0) ? 0.1 : (frame.timestamp - last_timestamp);
        last_timestamp = frame.timestamp;

        auto start = std::chrono::high_resolution_clock::now();
        dogm.updateGrid(frame, dt);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "Processing frame " << loader.getCurrentFrameIndex() << "/" << loader.getTotalFrames()
                  << ", Update time: " << duration.count() << " ms" << std::endl;
        
        const auto& grid_cells = dogm.getGridCells();
        int grid_size = dogm.getGridSize();

        for (int y = 0; y < grid_size; ++y) {
            for (int x = 0; x < grid_size; ++x) {
                const auto& cell = grid_cells[y * grid_size + x];
                float prob = pignistic(cell);
                if (prob > 0.1f && prob < 0.9f) {
                     output_file << std::fixed << std::setprecision(4) << frame.timestamp << ","
                                 << x << "," << y << "," << prob << ","
                                 << cell.mean_x_vel << "," << cell.mean_y_vel << "\n";
                }
            }
        }
    }

    std::cout << "Processing finished. Output saved to " << output_path << std::endl;
    output_file.close();

    return 0;
}
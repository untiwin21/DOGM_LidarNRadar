# DOGM CPU - Dynamic Occupancy Grid Map with LiDAR-Radar Fusion

CPU-based implementation of Dynamic Occupancy Grid Maps (DOGM) with LiDAR-Radar sensor fusion for indoor mobile robots.

## Features

- **Pure CPU Implementation**: No GPU dependencies, runs on standard Ubuntu 18.04 LTS
- **Sensor Fusion**: Combines LiDAR and Radar measurements for robust perception
- **Low-Speed Optimization**: Designed for indoor robots (< 1 m/s) and dynamic objects (< 3 m/s)
- **Real-Time Performance**: 10 Hz update rate on 3m x 3m grid with 10cm resolution
- **OpenMP Parallelization**: Utilizes multi-core CPUs for better performance

## System Requirements

- Ubuntu 18.04 LTS or later
- CMake 3.10+
- C++14 compatible compiler
- OpenCV 3.x or 4.x
- Eigen3
- OpenMP (usually included with GCC)

## Installation

### Dependencies on Ubuntu 18.04

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install build-essential cmake git

# Install OpenCV
sudo apt install libopencv-dev

# Install Eigen3
sudo apt install libeigen3-dev

# OpenMP is typically included with GCC

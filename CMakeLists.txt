cmake_minimum_required(VERSION 3.10)
project(DOGM_CPU LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Source files
set(DOGM_SOURCES
    src/dogm.cpp
    src/kernel/init.cpp
    src/kernel/predict.cpp
    src/kernel/update.cpp
    src/kernel/resampling.cpp
    src/kernel/sensor_fusion.cpp
)

# Create library
add_library(dogm_cpu STATIC ${DOGM_SOURCES})
target_link_libraries(dogm_cpu 
    ${OpenCV_LIBS}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
)

# Demo executable
add_executable(dogm_demo
    demo/main.cpp
    demo/data_loader.cpp
)

target_link_libraries(dogm_demo
    dogm_cpu
    ${OpenCV_LIBS}
)

# Enable OpenMP for parallel processing
if(OpenMP_CXX_FOUND)
    target_compile_options(dogm_cpu PUBLIC ${OpenMP_CXX_FLAGS})
endif()

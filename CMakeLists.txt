cmake_minimum_required(VERSION 3.10)
project(DOGM_Lidar_Radar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

add_library(dogm_cpu STATIC
    src/dogm.cpp
    src/kernel/init.cpp
    src/kernel/predict.cpp
    src/kernel/update.cpp
    src/kernel/resampling.cpp
    src/kernel/sensor_fusion.cpp
)

target_include_directories(dogm_cpu PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(dogm_cpu PUBLIC Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
    target_link_libraries(dogm_cpu PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(dogm_processor
    demo/progressor_main.cpp
    demo/data_loader.cpp
)

target_link_libraries(dogm_processor
    dogm_cpu
    ${OpenCV_LIBS}
)

add_executable(dogm_visualizer
    demo/visualizer_main.cpp
    demo/visualizer.cpp
)

target_link_libraries(dogm_visualizer
    ${OpenCV_LIBS}
)
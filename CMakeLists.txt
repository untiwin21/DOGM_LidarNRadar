cmake_minimum_required(VERSION 3.10)
project(DOGM_Lidar_Radar_Windows)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows 환경에서 vcpkg 등을 통해 패키지를 찾도록 설정
# 예: CMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP) # MSVC는 OpenMP를 기본 지원

# --- 1. dogm_cpu 핵심 라이브러리 ---
add_library(dogm_cpu STATIC
    src/dogm.cpp
    src/kernel/init.cpp
    src/kernel/predict.cpp
    src/kernel/update.cpp
    src/kernel/resampling.cpp
    src/kernel/sensor_fusion.cpp
)

# Public으로 해야 라이브러리를 사용하는 타겟에도 적용됨
target_include_directories(dogm_cpu PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(dogm_cpu PUBLIC Eigen3::Eigen)

# MSVC 컴파일러에서 OpenMP 활성화
if(MSVC AND OpenMP_CXX_FOUND)
    target_compile_options(dogm_cpu PUBLIC /openmp)
endif()


# --- 2. 데이터 처리 실행 파일 ---
add_executable(dogm_processor
    demo/processor_main.cpp
    demo/data_loader.cpp
)

target_link_libraries(dogm_processor
    dogm_cpu
    ${OpenCV_LIBS}
)


# --- 3. 시각화 실행 파일 ---
add_executable(dogm_visualizer
    demo/visualizer_main.cpp
    demo/visualizer.cpp
)

# visualizer는 OpenCV만 필요
target_link_libraries(dogm_visualizer
    ${OpenCV_LIBS}
)
# DOGM Lidar-Radar Fusion (CPU, Windows)

본 프로젝트는 논문 "A Random Finite Set Approach for Dynamic Occupancy Grid Maps"에 기반한 동적 점유 격자 지도(DOGM) 알고리즘을 CPU 환경에 맞게 재구현하고, Lidar와 Radar 센서 퓨전 기능을 추가한 것입니다.

기존의 실시간 시뮬레이션 코드에서 나아가, 사전 수집된 실제 주행 데이터를 처리하고 그 결과를 시각화하는 후처리(Post-processing) 파이프라인으로 재설계되었습니다. 모든 코드는 표준 C++로 작성되어 Windows 환경에서 실행됩니다.

![DOGM Visualization GIF](docs/videos/dogm_plot1.gif)
*(결과 예시 이미지: 원본 프로젝트의 시각화 결과)*

## 주요 특징

-   **Lidar-Radar 센서 퓨전**: Lidar의 정밀한 위치 정보와 Radar의 직접적인 속도 정보를 결합하여 동적 객체 탐지의 정확성과 신뢰도를 향상시킵니다.
-   **CPU 기반 병렬 처리**: CUDA 의존성을 완전히 제거하고, OpenMP를 사용하여 멀티코어 CPU 환경에서 높은 성능을 발휘하도록 최적화되었습니다.
-   **데이터 처리/시각화 파이프라인**:
    1.  **`dogm_processor`**: 실제 주행 데이터(Lidar, Radar, Odom)를 입력받아 DOGM 연산을 수행하고, 타임스탬프별 그리드 상태를 CSV 파일로 출력합니다.
    2.  **`dogm_visualizer`**: `dogm_processor`가 생성한 CSV 파일을 읽어 동영상 파일로 만들거나, 프레임 단위로 시각화합니다.
-   **Windows 플랫폼 지원**: Windows 및 MSVC(Visual Studio) 환경에서 빌드하고 실행할 수 있도록 CMake 빌드 시스템이 구성되었습니다.

## 시스템 요구사항

-   **운영체제**: Windows 10 이상
-   **컴파일러**: C++14 이상을 지원하는 컴파일러 (e.g., Visual Studio 2017 이상)
-   **빌드 시스템**: CMake 3.10 이상
-   **필수 라이브러리**:
    -   **OpenCV**: 데이터 시각화 및 영상 처리에 사용
    -   **Eigen3**: 행렬 및 벡터 연산에 사용

### 라이브러리 설치 (권장)

Windows 환경에서는 [vcpkg](https://github.com/microsoft/vcpkg)를 사용하여 라이브러리를 설치하는 것을 권장합니다. vcpkg를 설치한 후 아래 명령어를 실행하세요.

```powershell
.\vcpkg.exe install opencv:x64-windows eigen3:x64-windows
```

## 빌드 방법

1.  **저장소 클론:**
    ```bash
    git clone [https://github.com/Your-Username/DOGM_Lidar_Radar.git](https://github.com/Your-Username/DOGM_Lidar_Radar.git)
    cd DOGM_Lidar_Radar
    ```

2.  **CMake 프로젝트 생성 (vcpkg 사용 시):**
    ```bash
    mkdir build
    cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg_path]/scripts/buildsystems/vcpkg.cmake
    ```
    - `[vcpkg_path]`는 vcpkg가 설치된 경로로 변경해야 합니다.

3.  **빌드:**
    CMake가 생성한 Visual Studio 솔루션 (`.sln`) 파일을 열어 빌드하거나, 아래 명령어를 사용합니다.
    ```bash
    cmake --build . --config Release
    ```
    빌드가 완료되면 `build/Release` 또는 `build/Debug` 폴더 안에 `dogm_processor.exe`와 `dogm_visualizer.exe`가 생성됩니다.

## 사용 방법

프로젝트는 두 단계로 실행됩니다.

### 1단계: 데이터 처리 (`dogm_processor.exe`)

사전에 수집된 센서 데이터가 저장된 폴더를 입력으로 받아, DOGM 연산을 수행하고 결과를 CSV 파일로 저장합니다.

-   **입력 데이터 형식**: `data` 폴더에 프레임별로 `lidar_frame_XXX.txt`와 `radar_frame_XXX.txt` 파일이 있어야 합니다.
    -   `lidar_frame_XXX.txt`: 각 줄에 `각도(rad) 거리(m)` 형식
    -   `radar_frame_XXX.txt`: 각 줄에 `x(m) y(m) 반경방향속도(m/s)` 형식

-   **실행 명령어:**
    ```bash
    ./dogm_processor.exe <입력_데이터_폴더> <출력_CSV_파일명>
    ```

-   **실행 예시:**
    ```bash
    # Release 폴더에서 실행
    ./Release/dogm_processor.exe ../../data/sample output_dogm.csv
    ```
    실행이 완료되면 `output_dogm.csv` 파일이 생성됩니다.

### 2단계: 결과 시각화 (`dogm_visualizer.exe`)

`dogm_processor`가 생성한 `output_dogm.csv` 파일을 읽어 두 가지 모드로 시각화합니다.

#### 모드 1: 애니메이션 영상 생성

-   **설명**: 모든 프레임을 렌더링하여 하나의 `.mp4` 영상 파일로 저장합니다.
-   **실행 명령어:**
    ```bash
    ./dogm_visualizer.exe <입력_CSV_파일명> <그리드_셀_개수> --animate <출력_영상_파일명.mp4>
    ```
-   **실행 예시:**
    ```bash
    ./Release/dogm_visualizer.exe output_dogm.csv 100 --animate dogm_animation.mp4
    ```

#### 모드 2: 프레임별 보기

-   **설명**: 각 프레임을 0.5초 간격으로 화면에 표시합니다. (ESC 키로 종료)
-   **실행 명령어:**
    ```bash
    ./dogm_visualizer.exe <입력_CSV_파일명> <그리드_셀_개수> --view
    ```
-   **실행 예시:**
    ```bash
    ./Release/dogm_visualizer.exe output_dogm.csv 100 --view
    ```

---
**참고 논문:** Nuss, D., et al. "A Random Finite Set Approach for Dynamic Occupancy Grid Maps with Real-Time Application." (2016).
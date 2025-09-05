## DOGM with Lidar and Radar Fusion
### 1. 개요
이 프로젝트는 DOGM(Dynamic Occupancy Grid Map) 알고리즘을 C++로 구현한 것입니다. 특히 Lidar 센서와 Radar 센서의 데이터를 융합하여, 정적 환경 맵핑뿐만 아니라 동적 객체의 속도까지 추정하는 것을 목표로 합니다.

파티클 필터를 기반으로 각 그리드 셀의 점유 상태와 함께 내부 객체의 속도(v_x,v_y)를 추정하며, 이를 통해 주변 환경에 대한 더 풍부한 정보를 제공합니다.

### 2. 주요 특징
동적 환경 추적: 파티클 필터 기반의 DOGM을 사용하여 동적 환경을 실시간으로 추적합니다.

Lidar-Radar 센서 퓨전: Lidar의 정밀한 거리 정보를 통해 점유 확률을 계산하고, Radar의 방사 속도(radial velocity) 정보를 이용해 객체의 속도를 추정합니다.

신뢰도 기반 업데이트: Radar의 SNR(Signal-to-Noise Ratio) 값을 측정 신뢰도로 변환하여, 신뢰도 높은 데이터가 파티클 가중치 업데이트에 더 큰 영향을 미치도록 설계되었습니다.

4D 상태 추정: 각 파티클은 위치(x, y)와 속도(vx, vy)로 구성된 4차원 상태 벡터를 가집니다.

병렬 처리: OpenMP를 사용하여 파티클 업데이트, 그리드 계산 등 주요 연산 과정을 병렬화하여 처리 속도를 향상시켰습니다.

### 3. 의존성
CMake (3.10 이상)

C++17 이상을 지원하는 컴파일러 (e.g., GCC, Clang)

Eigen3 라이브러리

SFML 라이브러리 (Visualizer 실행 시 필요)

### 4. 빌드 방법
프로젝트를 빌드하기 위해 다음 명령어를 순서대로 실행합니다.

1. 프로젝트 클론
git clone <저장소_URL>
cd DOGM_LidarNRadar-main

2. 빌드 디렉토리 생성 및 이동
mkdir build
cd build

3. CMake 실행
cmake ..

4. 컴파일
make

빌드가 완료되면 build/bin/ 디렉토리 내에 dogm_progressor와 dogm_visualizer 두 개의 실행 파일이 생성됩니다.

### 5. 실행 방법
제공된 샘플 데이터는 data/sample 디렉토리에 있습니다.

#### 5.1. Progressor (데이터 처리 및 CSV 저장)
센서 데이터를 순차적으로 처리하고, 결과로 나온 동적 격자 지도를 CSV 파일로 저장합니다. GUI 없이 데이터 처리만 수행합니다.

형식: ./dogm_progressor <입력_데이터_디렉토리> <출력_CSV_파일_경로>
./bin/dogm_progressor ../data/sample ../output.csv

#### 5.2. Visualizer (실시간 시각화)
센서 데이터를 처리하는 과정을 실시간으로 시각화하여 보여줍니다.

형식: ./dogm_visualizer <입력_데이터_디렉토리>
./bin/dogm_visualizer ../data/sample

### 6. 입력 데이터 형식
data_loader는 특정 형식의 텍스트 파일을 읽도록 설계되었습니다.

Lidar 데이터 (LiDARMap_v2.txt)

timestamp x y intensity

각 행은 하나의 Lidar 포인트에 해당합니다.

Radar 데이터 (RadarMap_v2.txt)

timestamp x y velocity snr

각 행은 하나의 Radar 탐지 객체(detection)에 해당합니다.
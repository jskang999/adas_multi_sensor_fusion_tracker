# Multi Sensor Fusion Tracker (C++ / ADAS-style)

자율주행 / ADAS 센서 퓨전 포트폴리오용 C++ 프로젝트입니다.

- 2D 평면 상에서 여러 차량(Object)을 **Constant Velocity Kalman Filter**로 추적
- **Camera + Radar 센서 측정값**을 이용한 **센서 퓨전 + 다중 객체 추적**
- Camera: `z = [x, y]` 선형 측정
- Radar: `z = [r, angle, radial_velocity]` 비선형 측정 → Extended Kalman Filter
- Mahalanobis 거리 기반 데이터 연관 + greedy nearest-neighbor association
- 간단한 고속도로 시뮬레이터 + 센서 시뮬레이터 포함
- 결과를 CSV로 저장하고 Python 스크립트로 궤적 시각화

---

## Build

### Requirements

- Linux
- CMake >= 3.10
- C++17 compiler (g++ 등)
- Eigen3 (Ubuntu 예시)

  ```bash
  sudo apt-get install libeigen3-dev
  ```

### Build & Run

```bash
git clone <this-repo-url> multi_sensor_fusion_tracker
cd multi_sensor_fusion_tracker

mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=ON
make -j

# 실행 (인자: [num_objects] [num_steps])
./run_simulation 5 300
```

실행 후 아래 파일들이 생성됩니다.

- `ground_truth.csv`
- `tracks.csv`
- `detections.csv`

---

## Visualization

Python + matplotlib으로 궤적을 볼 수 있습니다.

```bash
cd ../tools
python3 plot_tracks.py
```

Ground truth 궤적과 추적된 트랙을 동시에 플롯합니다.

---

## Project Structure

```text
include/         # Public headers (API)
src/             # Library implementation
sim/             # Highway & sensor simulation
apps/            # Example applications
tools/           # Plotting / analysis scripts
docs/            # Design notes
```

`include/tracker.hpp` 의 `MultiSensorTracker` 클래스를 사용하면
다른 프로젝트에서도 센서 퓨전/트래킹 라이브러리처럼 쉽게 재사용할 수 있습니다.

---

## License

(원하는 라이선스를 여기에 추가)

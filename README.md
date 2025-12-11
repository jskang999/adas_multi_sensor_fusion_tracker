# Multi Sensor Fusion Tracker (C++ / ADAS-style)

자율주행 / ADAS 센서 퓨전 포트폴리오용 C++ 프로젝트입니다.

- 2D 평면 상에서 여러 차량(Object)을 **Constant Velocity Kalman Filter**로 추적
- **Camera + Radar 센서 측정값**을 이용한 **센서 퓨전 + 다중 객체 추적**
- Camera: `z = [x, y]` 선형 측정
- Radar: `z = [r, angle, radial_velocity]` 비선형 측정 → Extended Kalman Filter
- Mahalanobis 거리 기반 데이터 연관 + greedy nearest-neighbor association
- 간단한 고속도로 시뮬레이터 + 센서 시뮬레이터 포함
- 결과를 CSV로 저장하고 Python 스크립트로 궤적 및 이미지 오버레이 시각화

---

## 1. Top-level scripts (from project root)

프로젝트 루트에서 아래 네 개만 기억하면 됩니다.

- `./setup.sh`          → 의존성 설치 (C++ 빌드툴 + Eigen3 + Python + matplotlib)
- `./build.sh`          → CMake 빌드 + 시뮬레이션 실행 + CSV를 `output/`에 생성
- `./run_phthon_app.sh` → Python 시각화 실행 (플롯 + 이미지 오버레이)
- `./clean.sh`          → `build/`, `output/`, `__pycache__` 모두 정리

---

## 2. 의존성 설치 (Ubuntu 기준)

```bash
./setup.sh
```

이 스크립트가 다음을 처리합니다.

- `apt-get` 사용 가능한 경우:
  - `build-essential`, `cmake`, `libeigen3-dev`, `python3`, `python3-pip` 설치
- Python 패키지:
  - `pip3 install -r requirements.txt` 로 `matplotlib` 설치

---

## 3. 빌드 + 시뮬레이션 실행 (CSV 생성)

```bash
./build.sh
```

이 스크립트는 다음을 수행합니다.

1. `build/` 디렉토리 생성
2. CMake로 빌드 (`-DBUILD_EXAMPLES=ON`)
3. `build/run_simulation 5 300 ./output` 실행  
   → `output/ground_truth.csv`, `output/tracks.csv`, `output/detections.csv` 생성

---

## 4. Python 앱으로 시각화 실행

시뮬레이션이 한 번 돌아간 후, 최상위 디렉토리에서:

```bash
# 기본 이미지 (data/road.png)를 사용하는 경우
./run_phthon_app.sh

# 다른 이미지를 지정하고 싶은 경우
./run_phthon_app.sh path/to/your_image.png
```

`run_phthon_app.sh` 는 내부적으로 다음을 수행합니다.

1. `python3 tools/plot_tracks.py`  
   - `output/ground_truth.csv`, `output/tracks.csv`를 이용해 top-view 궤적 플롯
2. `python3 tools/visualize_image_with_tracks.py <image_path>`  
   - 지정한 이미지 위에 마지막 프레임의 GT/Track 위치를 오버레이  
   - 결과 이미지를 `output/visualization.png`로 저장

기본값 `image_path`는 `data/road.png` 이므로,  
프로젝트 내 `data/` 폴더에 `road.png`를 넣어 두면 바로 사용할 수 있습니다.

---

## 5. Clean (빌드/결과 정리)

빌드 아티팩트와 출력 파일을 모두 지우고 싶을 때:

```bash
./clean.sh
```

이 스크립트가 하는 일:

- `build/` 삭제
- `output/` 삭제
- 프로젝트 전체에서 `__pycache__` 디렉토리 삭제

---

## 6. Project Structure

```text
include/           # Public headers (API)
src/               # Library implementation
sim/               # Highway & sensor simulation
apps/              # Example applications (run_simulation)
tools/             # Plotting / analysis scripts (plot_tracks, visualize_image_with_tracks)
docs/              # Design notes
data/              # User-provided images for overlay (e.g., road.png)
output/            # (생성됨) simulation 결과 CSV 및 visualization.png
setup.sh           # 의존성 설치
build.sh           # 빌드 + 시뮬레이션 실행 (CSV 생성)
run_phthon_app.sh  # Python 시각화 실행 (플롯 + 이미지 오버레이)
clean.sh           # 빌드/출력/캐시 정리
```

`include/tracker.hpp` 의 `MultiSensorTracker` 클래스를 사용하면
다른 프로젝트에서도 센서 퓨전/트래킹 라이브러리처럼 쉽게 재사용할 수 있습니다.

---

## License

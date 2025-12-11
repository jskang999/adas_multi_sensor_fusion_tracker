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

## 1. 최상위에서 쓰는 스크립트 요약

프로젝트 루트에서 아래 세 개만 기억하면 됩니다.

- `./setup.sh`   → 의존성 설치 (C++ 빌드툴 + Eigen3 + Python + matplotlib)
- `./run_all.sh` → 빌드 + 시뮬레이션 실행 + 플롯까지 한 번에
- `./clean.sh`   → `build/`, `output/`, `__pycache__` 모두 정리

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

## 3. 빌드 + 시뮬레이션 + 플롯 한 번에 실행

```bash
./run_all.sh
```

이 스크립트는 다음을 수행합니다.

1. `build/` 디렉토리 생성
2. CMake로 빌드 및 `run_simulation` 실행  
   (인자: `5 300 <project_root>/output`)
3. 결과 CSV (`ground_truth.csv`, `tracks.csv`, `detections.csv`)를 `output/`에 생성
4. `python3 tools/plot_tracks.py`를 실행하여 궤적 플롯 (top-view)

---

## 4. 수동으로 빌드/실행하고 싶을 때

### Build

```bash
mkdir -p build
cd build
cmake .. -DBUILD_EXAMPLES=ON
make -j
```

### Run simulation

```bash
# 인자: [num_objects] [num_steps] [output_dir]
./run_simulation 5 300 ../output
```

### Plot

```bash
cd ..
python3 tools/plot_tracks.py
```

`tools/plot_tracks.py` 는 항상 **프로젝트 루트의 `output/` 디렉토리**에서  
`ground_truth.csv`, `tracks.csv`를 읽도록 작성되어 있습니다.

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
include/         # Public headers (API)
src/             # Library implementation
sim/             # Highway & sensor simulation
apps/            # Example applications (run_simulation)
tools/           # Plotting / analysis scripts
docs/            # Design notes
output/          # (생성됨) simulation 결과 CSV들
setup.sh         # 의존성 설치
run_all.sh       # 빌드 + 시뮬레이션 + 플롯 원샷 실행
clean.sh         # 빌드/출력/캐시 정리
```

`include/tracker.hpp` 의 `MultiSensorTracker` 클래스를 사용하면
다른 프로젝트에서도 센서 퓨전/트래킹 라이브러리처럼 쉽게 재사용할 수 있습니다.

---

## License

(원하는 라이선스를 여기에 추가)

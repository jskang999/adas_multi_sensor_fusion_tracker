#pragma once

#include <Eigen/Dense>

namespace msf {

enum class SensorType {
    Camera,
    Radar
};

using Vec4 = Eigen::Matrix<double, 4, 1>;
using Mat4 = Eigen::Matrix<double, 4, 4>;

struct Detection {
    SensorType sensor{SensorType::Camera};
    Eigen::VectorXd z;     // Camera: size 2, Radar: size 3
    double timestamp{0.0};
    double confidence{1.0};
};

struct TrackState {
    int id{-1};
    Vec4 x{Vec4::Zero()};  // [x, y, vx, vy]
    Mat4 P{Mat4::Identity()};
    bool confirmed{false};
    int age{0};            // total steps since creation
    int missed{0};         // consecutive missed detections
    double last_timestamp{0.0};
};

struct TrackerParams {
    // Process noise std (가속도 노이즈 등) - 대략적인 값
    double process_noise_std{1.0};

    // Camera 측정 노이즈 (x,y)
    double cam_pos_noise_std{1.0};

    // Radar 측정 노이즈 (r, angle, radial velocity)
    double radar_r_noise_std{1.0};
    double radar_angle_noise_std{0.05};
    double radar_vr_noise_std{0.5};

    // Mahalanobis 거리 게이트 (제곱값 기준으로 사용)
    double max_association_maha_dist{9.21}; // chi-square ~ 95% (2~3차원에 맞춰 대략)

    int max_missed{5};
    int min_hits_to_confirm{3};
};

} // namespace msf

#pragma once

#include "types.hpp"

namespace msf {

// Camera: z = [x, y]
Eigen::Vector2d camera_measurement(const Vec4& x);
Eigen::Matrix<double, 2, 4> camera_H();

// Radar: z = [r, angle, radial_velocity]
Eigen::Vector3d radar_measurement(const Vec4& x);
Eigen::Matrix<double, 3, 4> radar_H_jacobian(const Vec4& x);

// Radar 각도 차이를 [-pi, pi] 로 정규화
double normalize_angle(double angle);

} // namespace msf

#include "sensor_models.hpp"
#include <cmath>

namespace msf {

Eigen::Vector2d camera_measurement(const Vec4& x) {
    Eigen::Vector2d z;
    z << x(0), x(1);
    return z;
}

Eigen::Matrix<double, 2, 4> camera_H() {
    Eigen::Matrix<double, 2, 4> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    return H;
}

Eigen::Vector3d radar_measurement(const Vec4& x) {
    const double px = x(0);
    const double py = x(1);
    const double vx = x(2);
    const double vy = x(3);

    double rho = std::sqrt(px * px + py * py);
    double phi = std::atan2(py, px);
    double rho_dot = 0.0;

    if (rho > 1e-6) {
        rho_dot = (px * vx + py * vy) / rho;
    }

    Eigen::Vector3d z;
    z << rho, phi, rho_dot;
    return z;
}

Eigen::Matrix<double, 3, 4> radar_H_jacobian(const Vec4& x) {
    Eigen::Matrix<double, 3, 4> Hj;
    Hj.setZero();

    const double px = x(0);
    const double py = x(1);
    const double vx = x(2);
    const double vy = x(3);

    double c1 = px * px + py * py;
    double c2 = std::sqrt(c1);
    double c3 = c1 * c2;

    if (c1 < 1e-6) {
        Hj.setZero();
        return Hj;
    }

    // d(rho)/dx
    Hj(0, 0) = px / c2;
    Hj(0, 1) = py / c2;

    // d(phi)/dx
    Hj(1, 0) = -py / c1;
    Hj(1, 1) = px / c1;

    // d(rho_dot)/dx
    Hj(2, 0) = (vy * (px * py) - vx * py * py) / c3 + vx / c2;
    Hj(2, 1) = (vx * (px * py) - vy * px * px) / c3 + vy / c2;
    Hj(2, 2) = px / c2;
    Hj(2, 3) = py / c2;

    return Hj;
}

double normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

} // namespace msf

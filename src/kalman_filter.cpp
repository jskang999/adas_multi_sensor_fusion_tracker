#include "kalman_filter.hpp"

namespace msf {

KalmanFilter::KalmanFilter(int dim_x)
    : dim_x_(dim_x),
      x_(Eigen::VectorXd::Zero(dim_x)),
      P_(Eigen::MatrixXd::Identity(dim_x, dim_x)),
      F_(Eigen::MatrixXd::Identity(dim_x, dim_x)),
      Q_(Eigen::MatrixXd::Zero(dim_x, dim_x)) {}

void KalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void KalmanFilter::predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R) {
    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    P_ = (I - K * H) * P_;
}

} // namespace msf

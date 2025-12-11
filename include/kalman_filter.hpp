#pragma once

#include <Eigen/Dense>

namespace msf {

class KalmanFilter {
public:
    explicit KalmanFilter(int dim_x);

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    void setF(const Eigen::MatrixXd& F) { F_ = F; }
    void setQ(const Eigen::MatrixXd& Q) { Q_ = Q; }

    // F, Q를 이미 설정해뒀다는 가정 하에 predict
    void predict();

    // 선형 측정 업데이트
    void update(const Eigen::VectorXd& z,
                const Eigen::MatrixXd& H,
                const Eigen::MatrixXd& R);

    const Eigen::VectorXd& x() const { return x_; }
    const Eigen::MatrixXd& P() const { return P_; }

private:
    int dim_x_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
};

} // namespace msf

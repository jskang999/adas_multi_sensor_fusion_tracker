#include "tracker.hpp"
#include "sensor_models.hpp"
#include "data_association.hpp"

#include <Eigen/Dense>
#include <limits>
#include <algorithm>

namespace msf {

namespace {

// 프로세스 노이즈 Q 구성 (간단한 constant velocity 모델용)
Mat4 make_process_noise(double dt, double sigma_a) {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;

    Mat4 Q;
    Q.setZero();

    // 1D constant acceleration Q를 x,y에 각각 적용
    const double q11 = dt4 / 4.0;
    const double q13 = dt3 / 2.0;
    const double q31 = dt3 / 2.0;
    const double q33 = dt2;

    Q(0, 0) = q11;
    Q(0, 2) = q13;
    Q(2, 0) = q31;
    Q(2, 2) = q33;

    Q(1, 1) = q11;
    Q(1, 3) = q13;
    Q(3, 1) = q31;
    Q(3, 3) = q33;

    Q *= (sigma_a * sigma_a);
    return Q;
}

// 카메라 측정 노이즈 R (2x2)
Eigen::Matrix2d make_camera_R(double sigma_cam) {
    Eigen::Matrix2d R;
    R.setZero();
    R(0, 0) = sigma_cam * sigma_cam;
    R(1, 1) = sigma_cam * sigma_cam;
    return R;
}

// 레이더 측정 노이즈 R (3x3)
Eigen::Matrix3d make_radar_R(double sigma_r, double sigma_angle, double sigma_vr) {
    Eigen::Matrix3d R;
    R.setZero();
    R(0, 0) = sigma_r * sigma_r;
    R(1, 1) = sigma_angle * sigma_angle;
    R(2, 2) = sigma_vr * sigma_vr;
    return R;
}

// Mahalanobis 거리 제곱 계산
double mahalanobis_sq(const Eigen::VectorXd& y, const Eigen::MatrixXd& S) {
    Eigen::MatrixXd S_inv = S.inverse();
    double d2 = y.transpose() * S_inv * y;
    return d2;
}

} // anonymous namespace

MultiSensorTracker::MultiSensorTracker(const TrackerParams& params)
    : params_(params) {}

void MultiSensorTracker::predict(double timestamp) {
    for (auto& track : tracks_) {
        double dt = 0.0;
        if (track.last_timestamp > 0.0) {
            dt = timestamp - track.last_timestamp;
        }
        if (dt <= 0.0) {
            dt = 1e-3; // 너무 작은 dt 방지
        }

        // F 구성
        Mat4 F = Mat4::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        Mat4 Q = make_process_noise(dt, params_.process_noise_std);

        track.x = F * track.x;
        track.P = F * track.P * F.transpose() + Q;

        track.age += 1;
        track.last_timestamp = timestamp;
    }
}

void MultiSensorTracker::update(const std::vector<Detection>& detections) {
    const int n_tracks = static_cast<int>(tracks_.size());
    const int n_dets   = static_cast<int>(detections.size());

    if (n_tracks == 0) {
        // 모든 detection으로부터 새 track 생성
        for (const auto& det : detections) {
            create_track_from_detection(det);
        }
        return;
    }

    if (n_dets == 0) {
        // 모든 track missed 증가
        for (auto& track : tracks_) {
            track.missed += 1;
        }
    } else {
        // 비용 행렬 (Mahalanobis 거리 제곱)
        Eigen::MatrixXd cost(n_tracks, n_dets);
        cost.setConstant(std::numeric_limits<double>::infinity());

        Eigen::Matrix2d R_cam = make_camera_R(params_.cam_pos_noise_std);
        Eigen::Matrix3d R_rad = make_radar_R(params_.radar_r_noise_std,
                                             params_.radar_angle_noise_std,
                                             params_.radar_vr_noise_std);

        for (int i = 0; i < n_tracks; ++i) {
            const auto& track = tracks_[i];
            for (int j = 0; j < n_dets; ++j) {
                const auto& det = detections[j];

                if (det.sensor == SensorType::Camera) {
                    if (det.z.size() != 2) continue;
                    Eigen::Vector2d z = det.z.head<2>();
                    Eigen::Vector2d z_pred = camera_measurement(track.x);
                    Eigen::Matrix<double, 2, 4> H = camera_H();
                    Eigen::Vector2d y = z - z_pred;
                    Eigen::Matrix2d S = H * track.P * H.transpose() + R_cam;
                    double d2 = mahalanobis_sq(y, S);
                    cost(i, j) = d2;
                } else { // Radar
                    if (det.z.size() != 3) continue;
                    Eigen::Vector3d z = det.z.head<3>();
                    Eigen::Vector3d z_pred = radar_measurement(track.x);
                    Eigen::Matrix<double, 3, 4> H = radar_H_jacobian(track.x);

                    Eigen::Vector3d y = z - z_pred;
                    // 각도 차이 normalize
                    y(1) = normalize_angle(y(1));

                    Eigen::Matrix3d S = H * track.P * H.transpose() + R_rad;
                    double d2 = mahalanobis_sq(y, S);
                    cost(i, j) = d2;
                }
            }
        }

        double max_cost = params_.max_association_maha_dist;

        AssociationResult assoc = associate_greedy(cost, max_cost);

        // 먼저 모든 track를 missed로 가정
        for (auto& track : tracks_) {
            track.missed += 1;
        }

        // 매칭된 track 업데이트
        for (int i = 0; i < n_tracks; ++i) {
            int det_idx = assoc.track_assignment[i];
            if (det_idx < 0) continue;

            auto& track = tracks_[i];
            const auto& det = detections[det_idx];

            if (det.sensor == SensorType::Camera) {
                Eigen::Vector2d z = det.z.head<2>();
                Eigen::Vector2d z_pred = camera_measurement(track.x);
                Eigen::Matrix<double, 2, 4> H = camera_H();
                Eigen::Vector2d y = z - z_pred;
                Eigen::Matrix2d R = make_camera_R(params_.cam_pos_noise_std);
                Eigen::Matrix2d S = H * track.P * H.transpose() + R;
                Eigen::Matrix<double, 4, 2> K = track.P * H.transpose() * S.inverse();

                track.x = track.x + K * y;
                Mat4 I = Mat4::Identity();
                track.P = (I - K * H) * track.P;
            } else {
                Eigen::Vector3d z = det.z.head<3>();
                Eigen::Vector3d z_pred = radar_measurement(track.x);
                Eigen::Matrix<double, 3, 4> H = radar_H_jacobian(track.x);
                Eigen::Vector3d y = z - z_pred;
                y(1) = normalize_angle(y(1));
                Eigen::Matrix3d R = make_radar_R(params_.radar_r_noise_std,
                                                 params_.radar_angle_noise_std,
                                                 params_.radar_vr_noise_std);
                Eigen::Matrix3d S = H * track.P * H.transpose() + R;
                Eigen::Matrix<double, 4, 3> K = track.P * H.transpose() * S.inverse();

                track.x = track.x + K * y;
                Mat4 I = Mat4::Identity();
                track.P = (I - K * H) * track.P;
            }

            track.missed = 0;

            // hit 횟수 기반으로 confirmed 처리
            if (!track.confirmed && track.age >= params_.min_hits_to_confirm) {
                track.confirmed = true;
            }
        }

        // Unassigned detection → 새로운 track 생성
        for (int det_idx : assoc.unassigned_detections) {
            create_track_from_detection(detections[det_idx]);
        }
    }

    // 오래 missed 된 track 제거
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [&](const TrackState& t) {
                           return t.missed > params_.max_missed;
                       }),
        tracks_.end());
}

void MultiSensorTracker::create_track_from_detection(const Detection& det) {
    TrackState t;
    t.id = next_id_++;
    t.age = 1;
    t.missed = 0;
    t.confirmed = false;
    t.last_timestamp = det.timestamp;

    // 초기 상태 추정
    if (det.sensor == SensorType::Camera && det.z.size() >= 2) {
        double x = det.z(0);
        double y = det.z(1);
        t.x << x, y, 0.0, 0.0;
    } else if (det.sensor == SensorType::Radar && det.z.size() >= 3) {
        double r = det.z(0);
        double phi = det.z(1);
        double vr = det.z(2);
        double x = r * std::cos(phi);
        double y = r * std::sin(phi);
        double vx = vr * std::cos(phi);
        double vy = vr * std::sin(phi);
        t.x << x, y, vx, vy;
    } else {
        t.x.setZero();
    }

    // 초기 공분산
    t.P.setIdentity();
    t.P(0, 0) *= 10.0;
    t.P(1, 1) *= 10.0;
    t.P(2, 2) *= 10.0;
    t.P(3, 3) *= 10.0;

    tracks_.push_back(t);
}

} // namespace msf

#include "sensor_simulator.hpp"
#include "sensor_models.hpp"

#include <cmath>

namespace msf {

SensorSimulator::SensorSimulator(double cam_std,
                                 double radar_r_std,
                                 double radar_angle_std,
                                 double radar_vr_std,
                                 double detection_prob,
                                 double clutter_rate)
    : cam_std_(cam_std),
      radar_r_std_(radar_r_std),
      radar_angle_std_(radar_angle_std),
      radar_vr_std_(radar_vr_std),
      detection_prob_(detection_prob),
      clutter_rate_(clutter_rate),
      rng_(1234) {}

std::vector<Detection> SensorSimulator::generate(const std::vector<ObjectState>& objects,
                                                 double timestamp) {
    std::vector<Detection> detections;
    std::normal_distribution<double> cam_noise(0.0, cam_std_);
    std::normal_distribution<double> r_noise(0.0, radar_r_std_);
    std::normal_distribution<double> angle_noise(0.0, radar_angle_std_);
    std::normal_distribution<double> vr_noise(0.0, radar_vr_std_);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    // 실제 객체로부터 측정 생성
    for (const auto& obj : objects) {
        // Camera detection
        if (uni01(rng_) < detection_prob_) {
            Detection det;
            det.sensor = SensorType::Camera;
            det.timestamp = timestamp;
            det.z = Eigen::VectorXd(2);
            det.z(0) = obj.x + cam_noise(rng_);
            det.z(1) = obj.y + cam_noise(rng_);
            detections.push_back(det);
        }

        // Radar detection
        if (uni01(rng_) < detection_prob_) {
            double r = std::sqrt(obj.x * obj.x + obj.y * obj.y);
            double phi = std::atan2(obj.y, obj.x);
            double vr = (obj.x * obj.vx + obj.y * obj.vy) / (r + 1e-6);

            Detection det;
            det.sensor = SensorType::Radar;
            det.timestamp = timestamp;
            det.z = Eigen::VectorXd(3);
            det.z(0) = r + r_noise(rng_);
            det.z(1) = phi + angle_noise(rng_);
            det.z(2) = vr + vr_noise(rng_);
            detections.push_back(det);
        }
    }

    // Clutter (가짜 detection)
    int num_clutter = static_cast<int>(objects.size() * clutter_rate_);
    std::uniform_real_distribution<double> clutter_x(0.0, 120.0);
    std::uniform_real_distribution<double> clutter_y(-10.0, 10.0);

    for (int i = 0; i < num_clutter; ++i) {
        Detection det;
        det.sensor = SensorType::Camera; // 잡음은 카메라 잡음이라고 가정
        det.timestamp = timestamp;
        det.z = Eigen::VectorXd(2);
        det.z(0) = clutter_x(rng_);
        det.z(1) = clutter_y(rng_);
        det.confidence = 0.2;
        detections.push_back(det);
    }

    return detections;
}

} // namespace msf

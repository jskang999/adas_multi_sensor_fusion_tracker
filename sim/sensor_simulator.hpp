#pragma once

#include <vector>
#include <random>
#include "types.hpp"
#include "highway_scenario.hpp"

namespace msf {

class SensorSimulator {
public:
    SensorSimulator(double cam_std,
                    double radar_r_std,
                    double radar_angle_std,
                    double radar_vr_std,
                    double detection_prob = 0.9,
                    double clutter_rate   = 0.05);

    std::vector<Detection> generate(const std::vector<ObjectState>& objects,
                                    double timestamp);

private:
    double cam_std_;
    double radar_r_std_;
    double radar_angle_std_;
    double radar_vr_std_;
    double detection_prob_;
    double clutter_rate_;

    std::mt19937 rng_;
};

} // namespace msf

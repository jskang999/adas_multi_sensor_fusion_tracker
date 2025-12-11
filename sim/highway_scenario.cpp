#include "highway_scenario.hpp"
#include <random>

namespace msf {

HighwayScenario::HighwayScenario(int num_objects, double dt)
    : dt_(dt), time_(0.0) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> lane_dist(-3.0, 3.0);  // y
    std::uniform_real_distribution<double> x_dist(20.0, 80.0);    // 앞뒤 거리
    std::uniform_real_distribution<double> v_dist(10.0, 30.0);    // m/s

    objects_.reserve(num_objects);
    for (int i = 0; i < num_objects; ++i) {
        ObjectState obj;
        obj.id = i;
        obj.y = lane_dist(rng);
        obj.x = x_dist(rng) + i * 10.0;
        obj.vx = v_dist(rng);
        obj.vy = 0.0;
        objects_.push_back(obj);
    }
}

void HighwayScenario::step() {
    time_ += dt_;
    for (auto& obj : objects_) {
        obj.x += obj.vx * dt_;
        obj.y += obj.vy * dt_;
        // 간단하게 lane drift 같은 건 안 넣고 직진만
    }
}

} // namespace msf

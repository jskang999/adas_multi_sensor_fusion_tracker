#pragma once

#include <vector>

namespace msf {

struct ObjectState {
    int id{0};
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
};

class HighwayScenario {
public:
    HighwayScenario(int num_objects, double dt);

    void step();

    const std::vector<ObjectState>& objects() const { return objects_; }
    double time() const { return time_; }

private:
    double dt_;
    double time_{0.0};
    std::vector<ObjectState> objects_;
};

} // namespace msf

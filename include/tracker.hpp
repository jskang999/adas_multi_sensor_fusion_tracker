#pragma once

#include <vector>
#include "types.hpp"

namespace msf {

class MultiSensorTracker {
public:
    explicit MultiSensorTracker(const TrackerParams& params = TrackerParams{});

    // prediction은 timestamp 기준 (초 단위)
    void predict(double timestamp);

    // 현재 프레임의 모든 센서 측정 업데이트
    void update(const std::vector<Detection>& detections);

    const std::vector<TrackState>& get_tracks() const { return tracks_; }

private:
    TrackerParams params_;
    std::vector<TrackState> tracks_;
    int next_id_{0};

    void create_track_from_detection(const Detection& det);
};

} // namespace msf

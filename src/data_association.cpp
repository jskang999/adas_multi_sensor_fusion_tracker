#include "data_association.hpp"
#include <limits>
#include <tuple>
#include <algorithm>

namespace msf {

AssociationResult associate_greedy(const Eigen::MatrixXd& cost_matrix,
                                   double max_cost) {
    AssociationResult result;

    const int n_tracks = static_cast<int>(cost_matrix.rows());
    const int n_dets   = static_cast<int>(cost_matrix.cols());

    result.track_assignment.assign(n_tracks, -1);

    if (n_tracks == 0 || n_dets == 0) {
        for (int i = 0; i < n_tracks; ++i) {
            result.unassigned_tracks.push_back(i);
        }
        for (int j = 0; j < n_dets; ++j) {
            result.unassigned_detections.push_back(j);
        }
        return result;
    }

    struct Pair {
        int track;
        int det;
        double cost;
    };

    std::vector<Pair> pairs;
    pairs.reserve(n_tracks * n_dets);

    for (int i = 0; i < n_tracks; ++i) {
        for (int j = 0; j < n_dets; ++j) {
            double c = cost_matrix(i, j);
            if (std::isfinite(c) && c <= max_cost) {
                pairs.push_back({i, j, c});
            }
        }
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) {
                  return a.cost < b.cost;
              });

    std::vector<bool> track_used(n_tracks, false);
    std::vector<bool> det_used(n_dets, false);

    for (const auto& p : pairs) {
        if (!track_used[p.track] && !det_used[p.det]) {
            result.track_assignment[p.track] = p.det;
            track_used[p.track] = true;
            det_used[p.det] = true;
        }
    }

    for (int i = 0; i < n_tracks; ++i) {
        if (!track_used[i]) {
            result.unassigned_tracks.push_back(i);
        }
    }
    for (int j = 0; j < n_dets; ++j) {
        if (!det_used[j]) {
            result.unassigned_detections.push_back(j);
        }
    }

    return result;
}

} // namespace msf

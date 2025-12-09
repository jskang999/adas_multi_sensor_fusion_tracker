#pragma once

#include <Eigen/Dense>
#include <vector>

namespace msf {

struct AssociationResult {
    // track_assignment[i] = j → track i에 detection j 할당, 없으면 -1
    std::vector<int> track_assignment;
    std::vector<int> unassigned_tracks;
    std::vector<int> unassigned_detections;
};

// 비용 행렬(작을수록 좋은 cost)에 대해 greedy nearest-neighbor association
// max_cost 보다 크면 매칭하지 않음
AssociationResult associate_greedy(const Eigen::MatrixXd& cost_matrix,
                                   double max_cost);

} // namespace msf

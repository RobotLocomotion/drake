#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/planning/continuous_collision/options.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** One certification event: pair `pair_index` was certified over the
parameter interval [s_start, s_end] of segment `segment` from representative
configuration qc.
@ingroup planning_collision_checker */
struct CertificateRecord {
  int segment{};
  double s_start{};
  double s_end{};
  int pair_index{};
  Eigen::VectorXd qc;
  double phi_hat{};
  double motion_bound{};
  double threshold{};
};

/** Audit trail of every certification event of a run; an independent
replay (VerifyCertificate, declared in the api header) re-evaluates every
record and checks interval coverage of the full domain per pair.
@ingroup planning_collision_checker */
struct Certificate {
  std::vector<CertificateRecord> records;
  /** Pair table snapshot the indices refer to. */
  std::vector<PairId> pairs;
};

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

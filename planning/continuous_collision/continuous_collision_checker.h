#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/trajectory.h"
#include "drake/planning/continuous_collision/certificate.h"
#include "drake/planning/continuous_collision/distance_oracle.h"
#include "drake/planning/continuous_collision/motion_bound_table.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/continuous_collision/piecewise_bezier_path.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** Result of one certification call (the architecture). */
struct CertificationResult {
  Verdict verdict{};
  /** Earliest-first. */
  std::vector<Finding> findings;
  Statistics stats;
  /** Present iff Options::emit_certificate. */
  std::optional<Certificate> certificate;
};

/** Certifies — not samples — that a trajectory is collision-free over its
entire continuous time domain (the problem statement).

Guarantee: if a check returns Verdict::kCertifiedFree, then for every time t
in the trajectory's domain and every unfiltered geometry pair (A, B), the
signed distance φ_AB(q(t)) exceeds margin + padding(A, B) — under the stated
assumptions: exact real arithmetic up to the configured numerical slack, a
distance oracle accurate to its stated tolerance, and the geometry semantics
of the geometry-support scope (Mesh ≡ convex hull). This is a statement about
the continuum of configurations, not about samples. The certificate is a
property of the path, so retiming the trajectory afterwards does not invalidate
it.

Thread-compatible: the Check* methods are const, own no mutable state
outside per-call scratch, and are safe to call concurrently. */
class ContinuousCollisionChecker {
 public:
  struct Params {
    /** Plant + scene graph; the plant must be finalized. */
    std::shared_ptr<const drake::planning::RobotDiagram<double>> model;
    /** Per-body-pair padding, drake::planning::CollisionChecker semantics. */
    PaddingSpec padding{};
    Options default_options{};
  };

  /** Builds contexts, bounding spheres, topology tables, and runs the
  capability probe (throws on unsupported geometry pairs; the geometry-support
  scope). */
  explicit ContinuousCollisionChecker(Params params);

  ~ContinuousCollisionChecker();

  /** Certifies a trajectory (any supported Drake trajectory type). */
  CertificationResult CheckTrajectory(
      const drake::trajectories::Trajectory<double>& trajectory,
      const std::optional<Options>& options = {}) const;

  /** Certifies a piecewise-linear path through the given waypoint columns. */
  CertificationResult CheckPath(
      const Eigen::MatrixXd& waypoints,
      const std::optional<Options>& options = {}) const;

  /** Certifies the straight configuration-space edge q1 → q2. */
  CertificationResult CheckEdge(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      const std::optional<Options>& options = {}) const;

  /** Introspection / testing seams (all const, thread-safe). */
  PiecewiseBezierPath Normalize(
      const drake::trajectories::Trajectory<double>& trajectory,
      const std::optional<Options>& options = {}) const;
  MotionBoundTable ComputeMotionBounds(const PiecewiseBezierPath& path) const;
  const DistanceOracle& distance_oracle() const;
  const KinematicsEngine& kinematics_engine() const;
  const std::vector<PairRecord>& pairs() const;
  const drake::planning::RobotDiagram<double>& model() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/** Independently replays every record of `certificate` (recomputing node
control boxes from freshly restricted control points and re-querying
distances) and checks interval coverage of the full domain for every pair.
Returns true iff the certificate holds (the search algorithm). */
bool VerifyCertificate(const ContinuousCollisionChecker& checker,
                       const PiecewiseBezierPath& path,
                       const Certificate& certificate);

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

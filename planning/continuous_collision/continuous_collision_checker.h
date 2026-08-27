#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
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

/** Result of one certification call.
@ingroup planning_collision_checker */
struct CertificationResult {
  Verdict verdict{};
  /** Earliest-first. */
  std::vector<Finding> findings;
  Statistics stats;
  /** Present iff Options::emit_certificate. */
  std::optional<Certificate> certificate;
};

/** Certifies, rather than samples, that a trajectory is collision-free over
its entire continuous time domain.

Guarantee: if a check returns Verdict::kCertifiedFree, then for every time t in
the trajectory's domain and every unfiltered geometry pair (A, B), the signed
distance ϕ_AB(q(t)) exceeds margin + padding(A, B). That holds under three
assumptions: exact real arithmetic up to the configured numerical slack, a
distance oracle accurate to its stated tolerance, and Mesh ≡ convex hull. The
certificate is a property of the path, so retiming the trajectory afterwards
does not invalidate it.

Thread safety: the Check* methods are const, own no mutable state outside
per-call scratch, and may be called concurrently on one instance from arbitrary
threads. This is stronger than planning::CollisionChecker, whose documentation
requires a per-thread clone for use from threads the checker does not itself
own; no clone is needed here. Construction and destruction are not
thread-safe.
@ingroup planning_collision_checker */
class ContinuousCollisionChecker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContinuousCollisionChecker);

  struct Params {
    /** Plant + scene graph; the plant must be finalized. */
    std::shared_ptr<const RobotDiagram<double>> model;
    /** Per-body-pair padding; see PaddingSpec for the env/self rule. */
    PaddingSpec padding{};
    Options default_options{};
  };

  /** Builds contexts, bounding spheres and topology tables, and runs the
  capability probe.
  @throws std::exception if Params::model is null.
  @throws std::exception if the plant is not finalized.
  @throws std::exception if Params::default_options is invalid; see
  CheckTrajectory() for the conditions.
  @throws std::exception if PaddingSpec::per_body_pair is supplied and is not
  num_bodies × num_bodies, or if any pair's padding is not finite.
  @throws std::exception if the capability probe finds an unsupported pair;
  see DistanceOracle's constructor.
  @throws std::exception if the plant's topology or geometry defeats the
  motion bound; see KinematicsEngine's constructor. */
  explicit ContinuousCollisionChecker(Params params);

  ~ContinuousCollisionChecker();

  /** Certifies a trajectory (any supported Drake trajectory type).
  @throws std::exception if the trajectory cannot be normalized; see
  PiecewiseBezierPath::FromTrajectory().
  @throws std::exception if the trajectory's row count differs from the
  plant's number of generalized positions.
  @throws std::exception if Options::margin is not finite, if
  Options::query_tolerance or Options::certificate_slack is not a finite
  nonnegative distance, if Options::min_interval is outside (0, 1], if
  Options::max_reported_findings is below 1, or if Options::max_nodes is set
  to 0.
  @throws std::exception if margin + padding is negative for any pair; filter
  such a pair out instead of padding it below zero.
  @throws std::exception if the trajectory moves a coordinate of an
  unsupported joint type, or moves a HalfSpace across a rotational
  coordinate; see KinematicsEngine::ComputeMotionBoundTable(). */
  CertificationResult CheckTrajectory(
      const trajectories::Trajectory<double>& trajectory,
      const std::optional<Options>& options = {}) const;

  /** Certifies a piecewise-linear path through the given waypoint columns.
  @throws std::exception if `waypoints` has fewer than two columns.
  @throws std::exception if `waypoints` does not have one row per generalized
  position of the plant.
  @throws std::exception under every condition CheckTrajectory() lists. */
  CertificationResult CheckPath(
      const Eigen::MatrixXd& waypoints,
      const std::optional<Options>& options = {}) const;

  /** Certifies the straight configuration-space edge q1 → q2.
  @throws std::exception if q1 or q2 does not have one entry per generalized
  position of the plant.
  @throws std::exception under every condition CheckTrajectory() lists. */
  CertificationResult CheckEdge(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      const std::optional<Options>& options = {}) const;

  /** Converts `trajectory` to the internal piecewise-Bézier form, for
  introspection and testing. All const, and safe from arbitrary threads.
  @throws std::exception if the trajectory cannot be normalized; see
  PiecewiseBezierPath::FromTrajectory().
  @throws std::exception if the trajectory's row count differs from the
  plant's number of generalized positions. */
  PiecewiseBezierPath Normalize(
      const trajectories::Trajectory<double>& trajectory,
      const std::optional<Options>& options = {}) const;

  /** The λ table this checker would use for `path`, for introspection and
  testing.
  @throws std::exception if the path's row count differs from the plant's
  number of generalized positions.
  @throws std::exception if the path moves a coordinate of an unsupported
  joint type, or moves a HalfSpace across a rotational coordinate. */
  MotionBoundTable ComputeMotionBounds(const PiecewiseBezierPath& path) const;

  const DistanceOracle& distance_oracle() const;
  const KinematicsEngine& kinematics_engine() const;
  const std::vector<PairRecord>& pairs() const;
  const RobotDiagram<double>& model() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/** Independently replays every record of `certificate`, recomputing node
control boxes from freshly restricted control points and re-querying
distances, then checks interval coverage of the full domain for every pair.
@param checker      Supplies the model, the distance oracle and the pair
                    table the replay is checked against; the certificate must
                    have been produced by this checker.
@param path         The normalized path the certificate was produced for.
@param certificate  The audit trail to replay.
@returns true iff the certificate is a complete proof that `path` is free.
A certificate from a run that found a violation, ended inconclusive, exhausted
its node budget, or pruned the search leaves part of the domain uncovered, and
so returns false.
@ingroup planning_collision_checker */
bool VerifyCertificate(const ContinuousCollisionChecker& checker,
                       const PiecewiseBezierPath& path,
                       const Certificate& certificate);

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

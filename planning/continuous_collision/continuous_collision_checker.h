#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/parallelism.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** Outcome of one check.
@ingroup planning_collision_checker */
enum class Verdict {
  /** Proof: every unfiltered pair keeps signed distance > margin over the
  entire continuous time domain. */
  kCertifiedFree,
  /** An exactly-on-trajectory configuration violates the threshold. */
  kViolationFound,
  /** Some pair's clearance comes within Options::distance_resolution (plus
  the oracle tolerance) of the margin, so refining further cannot decide it:
  the trajectory grazes the margin. */
  kInconclusive,
};

/** Where the plan fails, or where it could not be decided.
@ingroup planning_collision_checker */
struct Finding {
  /** Trajectory time of the witness configuration. */
  double time{};
  /** The witness configuration, exactly on the trajectory. */
  Eigen::VectorXd q;
  geometry::GeometryId geometry_a;
  geometry::GeometryId geometry_b;
  multibody::BodyIndex body_a;
  multibody::BodyIndex body_b;
  /** Signed distance of the pair at q. */
  double distance{};
  /** Closest points in the world frame at q; present for violations, so that
  planners can push the trajectory out of collision. */
  std::optional<Eigen::Vector3d> nearest_a_W;
  std::optional<Eigen::Vector3d> nearest_b_W;
};

/** Options controlling one check.
@ingroup planning_collision_checker */
struct Options {
  /** Clearance margin δ in meters: the check certifies signed distance
  > margin for every unfiltered pair at every time. Must be finite and
  nonnegative. */
  double margin{0.0};
  /** Resolution floor r in meters. A pair stops being refined on a node once
  its bounded relative motion over that node is at most r; if it is still
  undecided there, the check reports Verdict::kInconclusive with that node's
  midpoint as the witness. Definitive verdicts are guaranteed for a trajectory
  whose clearance stays more than r (plus the oracle tolerance, see the class
  documentation) away from the margin everywhere; the cost of a grazing
  trajectory grows roughly linearly in 1/r. Must be finite and positive. */
  double distance_resolution{1e-6};
  /** Position coordinates whose junction continuity is checked modulo 2π
  (the GcsTrajectoryOptimization continuous-revolute convention).
  @see planning::trajectory_optimization::GetContinuousRevoluteJointIndices */
  std::vector<int> continuous_revolute_indices{};
  Parallelism parallelism{Parallelism::Max()};
};

/** Result of one check.
@ingroup planning_collision_checker */
struct Result {
  Verdict verdict{};
  /** The earliest violation, or the inconclusive witness; empty iff the
  verdict is Verdict::kCertifiedFree. */
  std::optional<Finding> finding;
  /** Nodes visited by the adaptive subdivision; a cost measure. */
  uint64_t num_nodes{0};
};

/** Certifies, rather than samples, that a trajectory is collision-free over
its entire continuous time domain.

Guarantee: if a check returns Verdict::kCertifiedFree, then for every time t in
the trajectory's domain and every unfiltered geometry pair (A, B), the signed
distance ϕ_AB(q(t)) exceeds Options::margin. That holds under three
assumptions: exact real arithmetic up to an internal numerical slack, a
distance oracle accurate to its stated tolerance, and Mesh ≡ convex hull. The
proof is a property of the path, so retiming the trajectory afterwards does not
invalidate it.

Resolution contract: write δ for Options::margin, r for
Options::distance_resolution, τ_p for the oracle tolerance of pair p (at least
1 µm; Drake's documented signed-distance accuracy for that shape combination),
ε for the internal slack (1 nm), and σ_p for the residual motion of coordinates
the trajectory holds constant only to within the continuity tolerance (exactly
zero when they are exactly constant, the common case). Then, for every pair,
 - if ϕ_p(q(t)) > δ + r + σ_p + 2τ_p + ε for every t, the pair is certified,
   so a trajectory that clears the margin by that much everywhere returns
   Verdict::kCertifiedFree;
 - if ϕ_p(q(t)) < δ − (r + σ_p + 2τ_p) for some t, the check returns
   Verdict::kViolationFound;
 - Verdict::kInconclusive is therefore possible only when some pair's
   clearance comes within that band of the margin, and its Finding then names
   an on-trajectory configuration whose reported distance lies in
   [δ − τ_p, δ + τ_p + ε + σ_p + r].
Resolutions below what double precision can represent along a segment are
capped by a floating-point backstop.

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

  /** Builds contexts, bounding spheres and topology tables, and runs the
  capability probe.
  @throws std::exception if `model` is null or its plant is not finalized.
  @throws std::exception if `default_options` is invalid; see CheckTrajectory().
  @throws std::exception if a pair's shape combination is unsupported, i.e. a
  deformable geometry or halfspace against halfspace.
  @throws std::exception if the plant's topology or geometry defeats the motion
  bound: a rotating HalfSpace, a reversed joint, a kinematic loop, or a
  proximity shape with no bounding sphere. */
  explicit ContinuousCollisionChecker(
      std::shared_ptr<const RobotDiagram<double>> model,
      const Options& default_options = {});

  ~ContinuousCollisionChecker();

  /** Certifies a trajectory (BezierCurve, BsplineTrajectory,
  PiecewisePolynomial, or a CompositeTrajectory of those).
  @throws std::exception if Options::margin is not a finite nonnegative
  distance, or if Options::distance_resolution is not a finite positive
  distance.
  @throws std::exception if the trajectory's row count differs from the
  plant's number of generalized positions.
  @throws std::exception if the trajectory is not one of the supported types,
  has a segment of degree above 10, or is discontinuous at a junction.
  @throws std::exception if Options::continuous_revolute_indices names a
  coordinate outside the plant's.
  @throws std::exception if the trajectory moves a coordinate of an
  unsupported joint type (quaternion floating, ball), or moves a HalfSpace
  across a rotational coordinate. */
  Result CheckTrajectory(const trajectories::Trajectory<double>& trajectory,
                         const std::optional<Options>& options = {}) const;

  /** Certifies the piecewise-linear path through the given waypoint columns.
  @throws std::exception if `waypoints` has fewer than two columns, or does not
  have one row per generalized position of the plant.
  @throws std::exception under every condition CheckTrajectory() lists. */
  Result CheckPath(const Eigen::MatrixXd& waypoints,
                   const std::optional<Options>& options = {}) const;

  /** Certifies the straight configuration-space edge q1 → q2.
  @throws std::exception if q1 or q2 does not have one entry per generalized
  position of the plant.
  @throws std::exception under every condition CheckTrajectory() lists. */
  Result CheckEdge(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
                   const std::optional<Options>& options = {}) const;

  const RobotDiagram<double>& model() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

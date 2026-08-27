#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/planning/continuous_collision/options.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** One Bézier segment q(s) = Σ_j B_{j,m}(s) P_j, s ∈ [0, 1] (trajectory
 * normalization).
 * @ingroup planning_collision_checker */
struct BezierSegment {
  /** Original time interval (bookkeeping only; the certificate is a property
  of the path and is invariant under time reparametrization). */
  double t_start{};
  double t_end{};
  /** n × (m+1); column j is control point P_j. */
  Eigen::MatrixXd control_points;
};

/** Ordered, C0-validated piecewise-Bézier path over the plant's generalized
positions. Every accepted trajectory type is converted, exactly, into this
representation up front (trajectory normalization).

Two Bézier facts the whole method rests on: (1) the curve lies in the convex
hull of its control points, so per coordinate i, q_i(s) ∈ [min_j P_{j,i},
max_j P_{j,i}]; (2) de Casteljau subdivision at any parameter u yields two
child curves whose control points exactly represent the two sub-curves and
are convex combinations of the parent's, so every descendant node's control
box is contained in this path's global control box. The apex of the de
Casteljau triangle at u is exactly q(u).
@ingroup planning_collision_checker */
class PiecewiseBezierPath {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseBezierPath);

  /** Normalizes any supported Drake trajectory (BezierCurve,
  CompositeTrajectory, BsplineTrajectory via knot insertion,
  PiecewisePolynomial via monomial→Bernstein change of basis).
  @throws std::exception on unsupported segment types, degree above
  options.max_conversion_degree, or junction discontinuity beyond
  options.continuity_tolerance (modulo 2π for coordinates in
  options.continuous_revolute_indices). */
  static PiecewiseBezierPath FromTrajectory(
      const trajectories::Trajectory<double>& trajectory,
      const Options& options);

  /** Normalizes an n × K waypoint matrix into K−1 order-1 segments (exact).
  Segment k spans time [k, k+1]. @throws std::exception if K < 2. */
  static PiecewiseBezierPath FromWaypoints(const Eigen::MatrixXd& waypoints,
                                           const Options& options);

  int num_positions() const { return num_positions_; }
  const std::vector<BezierSegment>& segments() const { return segments_; }
  double start_time() const { return segments_.front().t_start; }
  double end_time() const { return segments_.back().t_end; }

  /** Per-coordinate global control-point box over all segments (trajectory
  normalization); used for trajectory-adaptive prismatic reach bounds. */
  const Eigen::VectorXd& global_lower_bound() const { return global_lower_; }
  const Eigen::VectorXd& global_upper_bound() const { return global_upper_; }

  /** True for coordinates whose value is identical (within the continuity
  tolerance) across all control points of all segments; such coordinates are
  treated as welded for the check (trajectory normalization; the joint-support
  scope). */
  const std::vector<bool>& constant_coordinates() const {
    return constant_coordinates_;
  }

  /** Evaluates the path at time t (for tests and breakpoint checks; the hot
  loop never calls this — it uses de Casteljau apexes). */
  Eigen::VectorXd Value(double t) const;

  /** Evaluates segment `segment_index` at local parameter s ∈ [0, 1]. */
  Eigen::VectorXd EvaluateSegment(int segment_index, double s) const;

 private:
  PiecewiseBezierPath() = default;
  void FinalizeMetadata(double continuity_tolerance);

  int num_positions_{};
  std::vector<BezierSegment> segments_;
  Eigen::VectorXd global_lower_;
  Eigen::VectorXd global_upper_;
  std::vector<bool> constant_coordinates_;
};

/** Splits the Bézier control matrix `cps` (n × (m+1)) at u = 1/2 by de
Casteljau, writing the two children into `left` and `right` (resized as
needed) and the curve value at the midpoint (the apex) into `mid`.
Allocation-free when the outputs are already correctly sized.
@ingroup planning_collision_checker */
void DeCasteljauSplitAtHalf(const Eigen::MatrixXd& cps, Eigen::MatrixXd* left,
                            Eigen::MatrixXd* right, Eigen::VectorXd* mid);

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

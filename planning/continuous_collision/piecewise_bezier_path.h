#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

/* One Bézier segment q(s) = Σ_j B_{j,m}(s) P_j, s ∈ [0, 1]. */
struct BezierSegment {
  /* Original time interval (bookkeeping only; the proof is a property of the
  path and is invariant under time reparametrization). */
  double t_start{};
  double t_end{};
  /* n × (m+1); column j is control point P_j. */
  Eigen::MatrixXd control_points;
};

/* Ordered, C0-validated piecewise-Bézier path over the plant's generalized
positions. Every accepted trajectory type is converted, exactly, into this
representation up front.

Two Bézier facts the whole method rests on: (1) the curve lies in the convex
hull of its control points, so per coordinate i, q_i(s) ∈ [min_j P_{j,i},
max_j P_{j,i}]; (2) de Casteljau subdivision at any parameter u yields two
child curves whose control points exactly represent the two sub-curves and
are convex combinations of the parent's, so every descendant node's control
box is contained in this path's global control box. The apex of the de
Casteljau triangle at u is exactly q(u). */
class PiecewiseBezierPath {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseBezierPath);

  /* Normalizes any supported Drake trajectory (BezierCurve,
  CompositeTrajectory, BsplineTrajectory via knot insertion,
  PiecewisePolynomial via monomial→Bernstein change of basis).
  @throws std::exception on unsupported segment types, degree above
  kMaxConversionDegree, or junction discontinuity beyond kContinuityTolerance
  (modulo 2π for coordinates in `continuous_revolute_indices`). */
  static PiecewiseBezierPath FromTrajectory(
      const trajectories::Trajectory<double>& trajectory,
      const std::vector<int>& continuous_revolute_indices);

  /* Normalizes an n × K waypoint matrix into K−1 order-1 segments (exact).
  Segment k spans time [k, k+1].
  @throws std::exception if `waypoints` has fewer than two columns or zero
  rows. */
  static PiecewiseBezierPath FromWaypoints(const Eigen::MatrixXd& waypoints);

  int num_positions() const { return num_positions_; }
  const std::vector<BezierSegment>& segments() const { return segments_; }
  double start_time() const { return segments_.front().t_start; }
  double end_time() const { return segments_.back().t_end; }

  /* Per-coordinate global control-point box over all segments, used for
  trajectory-adaptive prismatic reach bounds. */
  const Eigen::VectorXd& global_lower_bound() const { return global_lower_; }
  const Eigen::VectorXd& global_upper_bound() const { return global_upper_; }

  /* True for coordinates whose value is identical (within
  kContinuityTolerance) across all control points of all segments; such
  coordinates are treated as welded for the check. */
  const std::vector<bool>& constant_coordinates() const {
    return constant_coordinates_;
  }

  /* Evaluates the path at time t, for tests and breakpoint checks; the hot
  loop uses de Casteljau apexes instead.
  @throws std::exception if t is outside [start_time(), end_time()], up to a
  parameter slack. */
  Eigen::VectorXd Value(double t) const;

  /* Evaluates segment `segment_index` at local parameter s ∈ [0, 1].
  @throws std::exception if segment_index is out of range, or if s is outside
  [0, 1] up to a parameter slack. */
  Eigen::VectorXd EvaluateSegment(int segment_index, double s) const;

 private:
  PiecewiseBezierPath() = default;
  void FinalizeMetadata();

  int num_positions_{};
  std::vector<BezierSegment> segments_;
  Eigen::VectorXd global_lower_;
  Eigen::VectorXd global_upper_;
  std::vector<bool> constant_coordinates_;
};

/* Splits the Bézier control matrix `cps` (n × (m+1)) at u = 1/2 by de
Casteljau, writing the two children into `left` and `right` (resized as
needed) and the curve value at the midpoint (the apex) into `mid`.
Allocation-free when the outputs are already correctly sized. */
void DeCasteljauSplitAtHalf(const Eigen::MatrixXd& cps, Eigen::MatrixXd* left,
                            Eigen::MatrixXd* right, Eigen::VectorXd* mid);

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

#include "drake/planning/continuous_collision/piecewise_bezier_path.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::NiceTypeName;
using drake::trajectories::BezierCurve;
using drake::trajectories::BsplineTrajectory;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

constexpr double kTwoPi = 6.2831853071795864769252867665590;

/* Relative slack when clamping an evaluation parameter back onto the closed
domain. Callers legitimately land a hair outside after their own arithmetic;
anything larger is a programming error and throws. */
constexpr double kParameterSlack = 1e-12;

/* Every conversion below inherits its breakpoints verbatim from the source
trajectory, so consecutive segments meet exactly in exact arithmetic; this
absorbs only round-off in the caller's own time bookkeeping. */
constexpr double kTimeContiguitySlack = 1e-9;

/* Pascal's triangle up to row `m`; table(j, a) = C(j, a) for a <= j, 0
otherwise. Exact in double for the degrees this file accepts (the default cap
is 10; C(10, 5) = 252). */
Eigen::MatrixXd BinomialTable(int m) {
  Eigen::MatrixXd table = Eigen::MatrixXd::Zero(m + 1, m + 1);
  for (int j = 0; j <= m; ++j) {
    table(j, 0) = 1.0;
    for (int a = 1; a <= j; ++a) {
      table(j, a) = table(j - 1, a - 1) + (a <= j - 1 ? table(j - 1, a) : 0.0);
    }
  }
  return table;
}

/* Converts one BsplineTrajectory into Bézier segments (trajectory
normalization, item 4).

Knot insertion (Boehm, via BsplineTrajectory::InsertKnots) raises every
distinct knot value inside the closed domain to multiplicity >= degree p. All
copies of a value are contiguous in a sorted knot vector, so afterwards every
nonempty span [t_i, t_{i+1}) satisfies t_{i-p+1} = ... = t_i and t_{i+1} = ...
= t_{i+p}; under exactly those conditions the p+1 basis functions active on the
span, N_{i-p}, ..., N_i, reduce to the Bernstein basis of degree p in
(t - t_i)/(t_{i+1} - t_i), so control points i-p ... i ARE that span's Bézier
control points. The conversion is exact in exact arithmetic; the acceptance
test in the test plan's T1 (1e-10 over >= 1e4 dense samples) guards the
indexing. */
void AppendBsplineSegments(const BsplineTrajectory<double>& bspline,
                           int source_index,
                           std::vector<BezierSegment>* segments) {
  if (bspline.cols() != 1) {
    throw std::runtime_error(fmt::format(
        "PiecewiseBezierPath: the BsplineTrajectory at segment index {} is "
        "{}x{}-valued; only column-vector-valued trajectories (cols() == 1) "
        "over the plant's generalized positions are supported.",
        source_index, bspline.rows(), bspline.cols()));
  }
  // InsertKnots mutates in place, so work on a copy of the caller's object.
  BsplineTrajectory<double> traj = bspline;
  const int order = traj.basis().order();
  const int degree = order - 1;

  if (degree > 0) {
    const double t0 = traj.basis().initial_parameter_value();
    const double tf = traj.basis().final_parameter_value();
    std::vector<double> additional_knots;
    const std::vector<double>& knots = traj.basis().knots();
    for (std::size_t i = 0; i < knots.size();) {
      std::size_t j = i;
      while (j < knots.size() && knots[j] == knots[i]) {
        ++j;
      }
      const int multiplicity = static_cast<int>(j - i);
      // Knots outside the domain do not bound any span we extract.
      if (knots[i] >= t0 && knots[i] <= tf) {
        for (int c = multiplicity; c < degree; ++c) {
          additional_knots.push_back(knots[i]);
        }
      }
      i = j;
    }
    if (!additional_knots.empty()) {
      traj.InsertKnots(additional_knots);
    }
  }

  const std::vector<double>& knots = traj.basis().knots();
  const int num_control_points = traj.num_control_points();
  const int num_positions = static_cast<int>(traj.rows());
  const std::size_t num_before = segments->size();
  for (int i = order - 1; i < num_control_points; ++i) {
    if (!(knots[i] < knots[i + 1])) {
      continue;  // Empty span, contributes no segment.
    }
    BezierSegment segment;
    segment.t_start = knots[i];
    segment.t_end = knots[i + 1];
    segment.control_points.resize(num_positions, order);
    for (int j = 0; j < order; ++j) {
      segment.control_points.col(j) = traj.control_points()[i - degree + j];
    }
    segments->push_back(std::move(segment));
  }
  if (segments->size() == num_before) {
    throw std::runtime_error(fmt::format(
        "PiecewiseBezierPath: the BsplineTrajectory at segment index {} has "
        "an empty parameter domain; a trajectory must span a positive time "
        "interval.",
        source_index));
  }
}

/* Converts one PiecewisePolynomial into Bézier segments (trajectory
normalization, item 5).

Drake stores each segment's polynomial in the monomial basis of the segment's
*relative* time tau = t - t_start. With s = tau/(t_end - t_start) in [0, 1] the
coefficients become alpha_a = c_a * (t_end - t_start)^a, and the exact monomial
-> Bernstein change of basis for a degree-m representation is

    s^a = sum_{j=a}^{m} [C(j, a) / C(m, a)] B_{j,m}(s),   hence
    P_j = sum_{a=0}^{j} [C(j, a) / C(m, a)] alpha_a.

The map is increasingly ill-conditioned in m, hence options.max_conversion_
degree. */
void AppendPiecewisePolynomialSegments(const PiecewisePolynomial<double>& pp,
                                       const Options& options, int source_index,
                                       std::vector<BezierSegment>* segments) {
  if (pp.cols() != 1) {
    throw std::runtime_error(fmt::format(
        "PiecewiseBezierPath: the PiecewisePolynomial at segment index {} is "
        "{}x{}-valued; only column-vector-valued trajectories (cols() == 1) "
        "over the plant's generalized positions are supported.",
        source_index, pp.rows(), pp.cols()));
  }
  const int num_positions = static_cast<int>(pp.rows());
  const int num_pp_segments = pp.get_number_of_segments();
  if (num_pp_segments < 1) {
    throw std::runtime_error(
        fmt::format("PiecewiseBezierPath: the PiecewisePolynomial at segment "
                    "index {} has no segments.",
                    source_index));
  }
  for (int k = 0; k < num_pp_segments; ++k) {
    int m = 0;
    for (int r = 0; r < num_positions; ++r) {
      m = std::max(m, pp.getSegmentPolynomialDegree(k, r, 0));
    }
    if (m > options.max_conversion_degree) {
      throw std::runtime_error(fmt::format(
          "PiecewiseBezierPath: PiecewisePolynomial segment {} (source segment "
          "index {}) has polynomial degree {}, above "
          "options.max_conversion_degree = {}. The monomial-to-Bernstein "
          "change of basis is ill-conditioned at high degree; either raise "
          "Options::max_conversion_degree deliberately or re-express the "
          "trajectory with more, lower-degree segments.",
          k, source_index, m, options.max_conversion_degree));
    }
    const double t_start = pp.start_time(k);
    const double t_end = pp.end_time(k);
    const double duration = t_end - t_start;
    if (!(duration > 0.0)) {
      throw std::runtime_error(
          fmt::format("PiecewiseBezierPath: PiecewisePolynomial segment {} "
                      "(source segment index {}) has non-positive duration {}.",
                      k, source_index, duration));
    }
    const Eigen::MatrixXd binomial = BinomialTable(m);
    BezierSegment segment;
    segment.t_start = t_start;
    segment.t_end = t_end;
    segment.control_points.setZero(num_positions, m + 1);
    Eigen::VectorXd alpha(m + 1);
    for (int r = 0; r < num_positions; ++r) {
      const Eigen::VectorXd coefficients =
          pp.getPolynomial(k, r, 0).GetCoefficients();
      const int degree = static_cast<int>(coefficients.size()) - 1;
      alpha.setZero();
      double scale = 1.0;
      for (int a = 0; a <= std::min(degree, m); ++a) {
        alpha[a] = coefficients[a] * scale;
        scale *= duration;
      }
      for (int j = 0; j <= m; ++j) {
        double sum = 0.0;
        for (int a = 0; a <= j; ++a) {
          sum += (binomial(j, a) / binomial(m, a)) * alpha[a];
        }
        segment.control_points(r, j) = sum;
      }
    }
    segments->push_back(std::move(segment));
  }
}

/* Dispatches `trajectory` by dynamic type and appends its Bézier segments,
recursing through CompositeTrajectory. `source_index` counts source segments
visited so far and appears in error messages (trajectory normalization, item 3).
*/
void AppendSegments(const Trajectory<double>& trajectory,
                    const Options& options, int* source_index,
                    std::vector<BezierSegment>* segments) {
  if (const auto* bezier =
          dynamic_cast<const BezierCurve<double>*>(&trajectory)) {
    if (bezier->control_points().cols() < 1) {
      throw std::runtime_error(
          fmt::format("PiecewiseBezierPath: the BezierCurve at segment index "
                      "{} has no control points.",
                      *source_index));
    }
    BezierSegment segment;
    segment.t_start = bezier->start_time();
    segment.t_end = bezier->end_time();
    segment.control_points = bezier->control_points();
    segments->push_back(std::move(segment));
    ++(*source_index);
    return;
  }
  if (const auto* composite =
          dynamic_cast<const CompositeTrajectory<double>*>(&trajectory)) {
    const int num = composite->get_number_of_segments();
    if (num < 1) {
      throw std::runtime_error(
          fmt::format("PiecewiseBezierPath: the CompositeTrajectory at "
                      "segment index {} has no segments.",
                      *source_index));
    }
    for (int i = 0; i < num; ++i) {
      AppendSegments(composite->segment(i), options, source_index, segments);
    }
    return;
  }
  if (const auto* bspline =
          dynamic_cast<const BsplineTrajectory<double>*>(&trajectory)) {
    AppendBsplineSegments(*bspline, *source_index, segments);
    ++(*source_index);
    return;
  }
  if (const auto* pp =
          dynamic_cast<const PiecewisePolynomial<double>*>(&trajectory)) {
    AppendPiecewisePolynomialSegments(*pp, options, *source_index, segments);
    ++(*source_index);
    return;
  }
  throw std::runtime_error(fmt::format(
      "PiecewiseBezierPath: unsupported trajectory type '{}' at segment index "
      "{}. Supported types are drake::trajectories::BezierCurve<double>, "
      "drake::trajectories::BsplineTrajectory<double>, "
      "drake::trajectories::PiecewisePolynomial<double>, and "
      "drake::trajectories::CompositeTrajectory<double> whose segments are "
      "themselves supported.",
      NiceTypeName::Get(trajectory), *source_index));
}

/* Checks shape, time ordering/contiguity and C0 junctions (trajectory
 * normalization). */
void ValidateSegments(int num_positions, const Options& options,
                      const std::vector<BezierSegment>& segments) {
  if (segments.empty()) {
    throw std::runtime_error(
        "PiecewiseBezierPath: the trajectory produced no Bézier segments.");
  }
  for (std::size_t i = 0; i < segments.size(); ++i) {
    const BezierSegment& segment = segments[i];
    if (segment.control_points.rows() != num_positions) {
      throw std::runtime_error(fmt::format(
          "PiecewiseBezierPath: segment {} has {} rows but the trajectory "
          "declares {} generalized positions; every segment must be valued in "
          "the same position space.",
          i, segment.control_points.rows(), num_positions));
    }
    if (segment.control_points.cols() < 1) {
      throw std::runtime_error(fmt::format(
          "PiecewiseBezierPath: segment {} has no control points.", i));
    }
    if (!(segment.t_end >= segment.t_start)) {
      throw std::runtime_error(fmt::format(
          "PiecewiseBezierPath: segment {} spans [{}, {}], which runs "
          "backwards in time.",
          i, segment.t_start, segment.t_end));
    }
    if (i > 0) {
      const double previous_end = segments[i - 1].t_end;
      const double slack =
          kTimeContiguitySlack *
          std::max({1.0, std::abs(previous_end), std::abs(segment.t_start)});
      if (std::abs(segment.t_start - previous_end) > slack) {
        throw std::runtime_error(
            fmt::format("PiecewiseBezierPath: segments are not contiguous "
                        "in time — segment {} ends at {} but segment {} "
                        "starts at {}. Segments must be ordered and meet "
                        "end-to-start.",
                        i - 1, previous_end, i, segment.t_start));
      }
    }
  }

  std::vector<bool> is_continuous_revolute(num_positions, false);
  for (int index : options.continuous_revolute_indices) {
    if (index < 0 || index >= num_positions) {
      throw std::runtime_error(fmt::format(
          "PiecewiseBezierPath: Options::continuous_revolute_indices contains "
          "{}, which is out of range for a trajectory with {} generalized "
          "positions.",
          index, num_positions));
    }
    is_continuous_revolute[index] = true;
  }

  // C0 junction check, per coordinate, modulo 2π for continuous-revolute
  // coordinates. A legitimate 2πk offset (GcsTrajectoryOptimization emits
  // these) is accepted and the control points are left exactly as they are:
  // forward kinematics is 2π-periodic in a revolute coordinate, so the
  // certificate is unaffected and re-aligning segments would be a no-op that
  // only risks introducing error (trajectory normalization).
  for (std::size_t i = 1; i < segments.size(); ++i) {
    const Eigen::MatrixXd& previous = segments[i - 1].control_points;
    const Eigen::MatrixXd& next = segments[i].control_points;
    for (int c = 0; c < num_positions; ++c) {
      const double raw_gap = next(c, 0) - previous(c, previous.cols() - 1);
      double gap = raw_gap;
      if (is_continuous_revolute[c]) {
        gap -= kTwoPi * std::round(gap / kTwoPi);
      }
      if (std::abs(gap) > options.continuity_tolerance) {
        const std::string modulo = is_continuous_revolute[c]
                                       ? fmt::format(" ({} modulo 2π)", gap)
                                       : "";
        throw std::runtime_error(fmt::format(
            "PiecewiseBezierPath: C0 discontinuity at the junction between "
            "segments {} and {} in coordinate {}: the gap is {}{}, which "
            "exceeds Options::continuity_tolerance = {}. A discontinuous "
            "trajectory teleports; per-segment certificates would not cover "
            "the jump. If coordinate {} is a continuous revolute joint, list "
            "it in Options::continuous_revolute_indices.",
            i - 1, i, c, raw_gap, modulo, options.continuity_tolerance, c));
      }
    }
  }
}

}  // namespace

PiecewiseBezierPath PiecewiseBezierPath::FromTrajectory(
    const Trajectory<double>& trajectory, const Options& options) {
  if (trajectory.cols() != 1) {
    throw std::runtime_error(fmt::format(
        "PiecewiseBezierPath::FromTrajectory: the trajectory is {}x{}-valued; "
        "only column-vector-valued trajectories (cols() == 1) over the plant's "
        "generalized positions are supported.",
        trajectory.rows(), trajectory.cols()));
  }
  const int num_positions = static_cast<int>(trajectory.rows());
  if (num_positions < 1) {
    throw std::runtime_error(
        "PiecewiseBezierPath::FromTrajectory: the trajectory has zero rows; "
        "expected one row per generalized position.");
  }

  PiecewiseBezierPath path;
  path.num_positions_ = num_positions;
  int source_index = 0;
  AppendSegments(trajectory, options, &source_index, &path.segments_);
  ValidateSegments(num_positions, options, path.segments_);
  path.FinalizeMetadata(options.continuity_tolerance);
  return path;
}

PiecewiseBezierPath PiecewiseBezierPath::FromWaypoints(
    const Eigen::MatrixXd& waypoints, const Options& options) {
  if (waypoints.rows() < 1) {
    throw std::runtime_error(
        "PiecewiseBezierPath::FromWaypoints: the waypoint matrix has zero "
        "rows; expected one row per generalized position.");
  }
  if (waypoints.cols() < 2) {
    throw std::runtime_error(fmt::format(
        "PiecewiseBezierPath::FromWaypoints: at least 2 waypoints (columns) "
        "are required to form a path; got {}.",
        waypoints.cols()));
  }
  const int num_positions = static_cast<int>(waypoints.rows());
  const int num_segments = static_cast<int>(waypoints.cols()) - 1;

  PiecewiseBezierPath path;
  path.num_positions_ = num_positions;
  path.segments_.reserve(num_segments);
  for (int k = 0; k < num_segments; ++k) {
    // A straight waypoint-to-waypoint move is exactly the order-1 Bézier with
    // control points {q_k, q_{k+1}} (trajectory normalization, item 1). Segment
    // k spans the nominal time interval [k, k+1]; the certificate does not
    // depend on the time parametrization.
    BezierSegment segment;
    segment.t_start = k;
    segment.t_end = k + 1;
    segment.control_points.resize(num_positions, 2);
    segment.control_points.col(0) = waypoints.col(k);
    segment.control_points.col(1) = waypoints.col(k + 1);
    path.segments_.push_back(std::move(segment));
  }
  ValidateSegments(num_positions, options, path.segments_);
  path.FinalizeMetadata(options.continuity_tolerance);
  return path;
}

void PiecewiseBezierPath::FinalizeMetadata(double continuity_tolerance) {
  const int n = num_positions_;
  global_lower_ =
      Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
  global_upper_ =
      Eigen::VectorXd::Constant(n, -std::numeric_limits<double>::infinity());
  for (const BezierSegment& segment : segments_) {
    global_lower_ =
        global_lower_.cwiseMin(segment.control_points.rowwise().minCoeff());
    global_upper_ =
        global_upper_.cwiseMax(segment.control_points.rowwise().maxCoeff());
  }
  // By the convex-hull property the curve never leaves [global_lower_,
  // global_upper_], so a coordinate whose whole control-point range collapses
  // to within the continuity tolerance cannot move on this path and is
  // treated as welded (trajectory normalization; the joint-support scope).
  constant_coordinates_.assign(n, false);
  for (int i = 0; i < n; ++i) {
    constant_coordinates_[i] =
        (global_upper_[i] - global_lower_[i]) <= continuity_tolerance;
  }
}

Eigen::VectorXd PiecewiseBezierPath::Value(double t) const {
  DRAKE_DEMAND(!segments_.empty());
  const double t0 = start_time();
  const double tf = end_time();
  const double slack =
      kParameterSlack * std::max({1.0, std::abs(t0), std::abs(tf)});
  if (!(t >= t0 - slack) || !(t <= tf + slack)) {
    throw std::runtime_error(
        fmt::format("PiecewiseBezierPath::Value: time {} is outside the path's "
                    "domain [{}, {}].",
                    t, t0, tf));
  }
  const double clamped = std::clamp(t, t0, tf);
  // Last segment whose start time is at or before `clamped`. At an interior
  // junction the later segment wins, matching
  // drake::trajectories::PiecewiseTrajectory::get_segment_index(). The choice
  // is observable only when a junction carries a legitimate 2πk offset in a
  // continuous-revolute coordinate, where the two sides are different
  // representatives of the same configuration (trajectory normalization).
  int low = 0;
  int high = static_cast<int>(segments_.size()) - 1;
  while (low < high) {
    const int mid = low + (high - low + 1) / 2;
    if (segments_[mid].t_start <= clamped) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  const BezierSegment& segment = segments_[low];
  const double duration = segment.t_end - segment.t_start;
  const double s =
      (duration > 0.0) ? (clamped - segment.t_start) / duration : 0.0;
  return EvaluateSegment(low, std::clamp(s, 0.0, 1.0));
}

Eigen::VectorXd PiecewiseBezierPath::EvaluateSegment(int segment_index,
                                                     double s) const {
  if (segment_index < 0 ||
      segment_index >= static_cast<int>(segments_.size())) {
    throw std::runtime_error(fmt::format(
        "PiecewiseBezierPath::EvaluateSegment: segment index {} is out of "
        "range; the path has {} segments.",
        segment_index, segments_.size()));
  }
  if (!(s >= -kParameterSlack) || !(s <= 1.0 + kParameterSlack)) {
    throw std::runtime_error(
        fmt::format("PiecewiseBezierPath::EvaluateSegment: parameter s = {} "
                    "is outside the segment's domain [0, 1].",
                    s));
  }
  const double u = std::clamp(s, 0.0, 1.0);
  const Eigen::MatrixXd& control_points =
      segments_[segment_index].control_points;
  const int m = static_cast<int>(control_points.cols()) - 1;
  // de Casteljau: repeated convex combinations, so the evaluation never leaves
  // the convex hull of the control points and is numerically stable.
  Eigen::MatrixXd work = control_points;
  for (int r = 1; r <= m; ++r) {
    for (int j = 0; j <= m - r; ++j) {
      work.col(j) = (1.0 - u) * work.col(j) + u * work.col(j + 1);
    }
  }
  return work.col(0);
}

void DeCasteljauSplitAtHalf(const Eigen::MatrixXd& cps, Eigen::MatrixXd* left,
                            Eigen::MatrixXd* right, Eigen::VectorXd* mid) {
  DRAKE_THROW_UNLESS(left != nullptr);
  DRAKE_THROW_UNLESS(right != nullptr);
  DRAKE_THROW_UNLESS(mid != nullptr);
  const int n = static_cast<int>(cps.rows());
  const int m = static_cast<int>(cps.cols()) - 1;
  DRAKE_THROW_UNLESS(m >= 0);
  // Eigen's resize() is a no-op when the size already matches, so a caller
  // that pre-sizes the outputs pays no allocation here (the performance
  // requirements, P1).
  if (left->rows() != n || left->cols() != m + 1) {
    left->resize(n, m + 1);
  }
  if (right->rows() != n || right->cols() != m + 1) {
    right->resize(n, m + 1);
  }
  if (mid->size() != n) {
    mid->resize(n);
  }

  // de Casteljau at u = 1/2 with the triangle b_j^r built in place inside
  // `right`: b_j^r = (b_j^{r-1} + b_{j+1}^{r-1})/2 for j = 0 ... m-r. Sweeping
  // j upward is safe because entry j+1 is not written until the next j. The
  // left child's control points are the first entries of each triangle row,
  // b_0^r; the right child's are the last entries, b_{m-r}^r, which is exactly
  // the entry the sweep leaves at column m-r; and the apex b_0^m = q(1/2) is
  // the right child's first control point (trajectory normalization; the search
  // algorithm).
  *right = cps;
  left->col(0) = cps.col(0);
  for (int r = 1; r <= m; ++r) {
    for (int j = 0; j <= m - r; ++j) {
      right->col(j) = 0.5 * (right->col(j) + right->col(j + 1));
    }
    left->col(r) = right->col(0);
  }
  *mid = right->col(0);
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

#include "drake/automotive/maliput/rndf/spline_helpers.h"

#include <algorithm>
#include <limits>
#include <utility>

#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector4.hh>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace maliput {
namespace rndf {

// The InverseFunctionInterpolator holds a graph structure where each node
// is a linear interpolant. The graph has kFunctionPartitionTreeDegree on each
// level and the depth is controlled by kFunctionPartitionTreeMaxDepth. These
// two numbers are related with the error_boundary provided to
// InverseFunctionInterpolator's constructor in the sense that we can get
// smaller error bounds given higher values of kFunctionPartitionTreeDegree
// and / or kFunctionPartitionTreeMaxDepth.
static const int kFunctionPartitionTreeDegree = 10;
static const int kFunctionPartitionTreeMaxDepth = 10;

// A Cubic Bezier coefficient matrix.
static constexpr double kBezierMatrix[16] = {
    1.0, 0.0, 0.0, 0.0,
    -3.0, 3.0, 0.0, 0.0,
    3.0, -6.0, 3.0, 0.0,
    -1.0, 3.0, -3.0, 1.0};

// A Hermite Spline coefficient matrix.
static constexpr double kHermiteMatrix[16] = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    -3.0, -2.0, 3.0, -1.0,
    2.0, 1.0, -2.0, 1.0};

InverseFunctionInterpolator::InverseFunctionInterpolator(
    std::function<double(double)> function, double xmin, double xmax,
    double error_boundary)
    : function_(function),
      error_boundary_(error_boundary),
      partition_tree_degree_(kFunctionPartitionTreeDegree),
      partition_tree_max_depth_(kFunctionPartitionTreeMaxDepth) {
  // Make sure the error boundary is attainable.
  DRAKE_THROW_UNLESS(error_boundary > 0);
  // Make sure that xmin is smaller that xmax
  DRAKE_THROW_UNLESS(xmin < xmax);
  // Instantiate the partition tree root.
  partition_tree_ =
      std::make_unique<InverseFunctionInterpolator::FunctionGraphPartition>();
  partition_tree_->segment.xmin = xmin;
  partition_tree_->segment.xmax = xmax;
  partition_tree_->segment.ymin = function(xmin);
  partition_tree_->segment.ymax = function(xmax);
}

double InverseFunctionInterpolator::InterpolateMthDerivative(
    int derivative_order, double y) {
  // Make sure that the derivative order is non-negative.
  DRAKE_THROW_UNLESS(derivative_order >= 0);
  // Make sure that y is not above x(y) image interval upper bound.
  DRAKE_THROW_UNLESS((y - partition_tree_->segment.ymax) < error_boundary_);
  // Make sure that y is not below x(y) image interval low bound.
  DRAKE_THROW_UNLESS((partition_tree_->segment.ymin - y) < error_boundary_);

  // Zero out any derivative of higher order than 1, as this is a piecewise
  // linear interpolant.
  if (derivative_order > 1) return 0.0;

  // Get partition tree root.
  InverseFunctionInterpolator::FunctionGraphPartition* root =
      partition_tree_.get();

  // Clamp interpolation y.
  const double iy =
      std::min(std::max(y, root->segment.ymin), root->segment.ymax);

  // Traverse down the partition tree until the required error is attained.
  int depth = 0;
  while (true) {
    if (root->partitions.empty()) {
      // Visiting a leaf node.

      // Linearly interpolate x for the given y.
      const double dx = root->segment.xmax - root->segment.xmin;
      const double dy = root->segment.ymax - root->segment.ymin;
      const double ix =
          root->segment.xmin + (dx / dy) * (iy - root->segment.ymin);

      // Check if the interpolation meets the error bound requirements.
      if (std::abs(function_(ix) - iy) < error_boundary_) {
        if (derivative_order == 1) {
          // Return derivative dx/dy at y.
          return dx / dy;
        }
        // Return interpolated x(y).
        return ix;
      }

      // Safety check before building next tree level.
      DRAKE_DEMAND(depth <= partition_tree_max_depth_);

      // Build sub-partitions for this partition.
      const double segmentdx = dx / partition_tree_degree_;
      for (int i = 0; i < partition_tree_degree_; ++i) {
        const double segmentxmin = root->segment.xmin + i * segmentdx;
        const double segmentxmax = segmentxmin + segmentdx;

        auto subp = std::make_unique<
            InverseFunctionInterpolator::FunctionGraphPartition>();
        subp->segment.xmin = segmentxmin;
        subp->segment.xmax = segmentxmax;
        subp->segment.ymin = function_(segmentxmin);
        subp->segment.ymax = function_(segmentxmax);
        root->partitions.push_back(std::move(subp));
      }
    }
    // Visiting an inner node.

    // Search subpartition that contains y.
    auto it = std::find_if(
        root->partitions.begin(), root->partitions.end(),
        [&iy](const std::unique_ptr<
              InverseFunctionInterpolator::FunctionGraphPartition>& subp) {
          return (subp->segment.ymin <= iy && iy <= subp->segment.ymax);
        });

    // Use subpartition as new root.
    DRAKE_DEMAND(it != root->partitions.end());
    DRAKE_DEMAND(it->get() != nullptr);

    root = it->get();
    depth++;
  }
}

ArcLengthParameterizedSpline::ArcLengthParameterizedSpline(
    std::unique_ptr<ignition::math::Spline> spline, double error_boundary)
    : q_t_(std::move(spline)) {
  DRAKE_THROW_UNLESS(q_t_ != nullptr);
  // Instantiate an inverse function interpolator
  // for the given spline path length function.
  F_ts_ = std::make_unique<InverseFunctionInterpolator>(
      [this](double t) { return q_t_->ArcLength(t); }, 0.0, 1.0,
      error_boundary);
}

ignition::math::Vector3d ArcLengthParameterizedSpline::InterpolateMthDerivative(
    int derivative_order, double s) {
  DRAKE_THROW_UNLESS(derivative_order >= 0);
  if (derivative_order > 3) {
    // derivative_order > 3 => p = 0 (as this is a cubic interpolator)
    return ignition::math::Vector3d(0.0, 0.0, 0.0);
  }
  const double t_s = F_ts_->InterpolateMthDerivative(0, s);
  if (derivative_order == 0) {
    // derivative_order = 0 => P(s) = Q(t(s))
    return q_t_->InterpolateMthDerivative(0, t_s);
  }
  const double t_prime_s = F_ts_->InterpolateMthDerivative(1, s);
  ignition::math::Vector3d q_prime_t = q_t_->InterpolateMthDerivative(1, t_s);

  if (derivative_order == 1) {
    // We need to apply chain rule to get the derivative of Q(t(s)) with respect
    // to s.
    // derivative_order = 1 => P'(s) = Q'(t(s)) * t'(s)
    return q_prime_t * t_prime_s;
  }
  const double t_prime_s_2 = t_prime_s * t_prime_s;
  const double t_prime2_s = F_ts_->InterpolateMthDerivative(2, s);
  const ignition::math::Vector3d q_prime2_t =
      q_t_->InterpolateMthDerivative(2, t_s);
  if (derivative_order == 2) {
    // We need to apply chain rule to get the second of derivative of Q(t(s))
    // with respect to s.
    // derivative_order = 2 => P''(s) = Q''(t(s)) * t'(s)^2 + Q'(t(s)) * t''(s)
    return q_prime2_t * t_prime_s_2 + q_prime_t * t_prime2_s;
  }
  const double t_prime_s_3 = t_prime_s_2 * t_prime_s;
  const double t_prime3_s = F_ts_->InterpolateMthDerivative(3, s);
  const ignition::math::Vector3d q_prime3_t =
      q_t_->InterpolateMthDerivative(3, t_s);
  // We need to apply chain rule to get the third derivative of Q(t(s))
  // with respect to s.
  // derivative_order = 3 => P'''(s) = Q'''(t(s)) * t'(s)^3
  //                                   + 3 * Q''(t(s)) * t'(s) * t''(s)
  //                                   + Q'(t(s)) * t'''(s)
  return (q_prime3_t * t_prime_s_3 + 3 * q_prime2_t * t_prime_s * t_prime2_s +
          q_prime_t * t_prime3_s);
}

double ArcLengthParameterizedSpline::FindClosestPointTo(
    const ignition::math::Vector3d& point, double step) const {
  double closest_s = 0.0;
  double min_distance = std::numeric_limits<double>::max();
  double distance = 0.0;
  double t_s = 0.0;
  for (double s = 0.0; s < q_t_->ArcLength(); s += step) {
    t_s = F_ts_->InterpolateMthDerivative(0, s);
    distance = (q_t_->InterpolateMthDerivative(0, t_s) - point).Length();
    if (distance < min_distance) {
      min_distance = distance;
      closest_s = s;
    }
  }
  t_s = F_ts_->InterpolateMthDerivative(0, q_t_->ArcLength());
  distance = (q_t_->InterpolateMthDerivative(0, t_s) - point).Length();
  if (distance < min_distance) {
    min_distance = distance;
    closest_s = q_t_->ArcLength();
  }

  return closest_s;
}

namespace {

// @returns An ignition::math::Matrix4d object with kBezierMatrix coefficients.
ignition::math::Matrix4d BezierBasis() {
  return ignition::math::Matrix4d(
      kBezierMatrix[0], kBezierMatrix[1], kBezierMatrix[2], kBezierMatrix[3],
      kBezierMatrix[4], kBezierMatrix[5], kBezierMatrix[6], kBezierMatrix[7],
      kBezierMatrix[8], kBezierMatrix[9], kBezierMatrix[10], kBezierMatrix[11],
      kBezierMatrix[12], kBezierMatrix[13], kBezierMatrix[14],
      kBezierMatrix[15]);
}

// @returns An ignition::math::Matrix4d object with kHermiteMatrix coefficients.
ignition::math::Matrix4d HermiteBasis() {
  return ignition::math::Matrix4d(
      kHermiteMatrix[0], kHermiteMatrix[1], kHermiteMatrix[2],
      kHermiteMatrix[3], kHermiteMatrix[4], kHermiteMatrix[5],
      kHermiteMatrix[6], kHermiteMatrix[7], kHermiteMatrix[8],
      kHermiteMatrix[9], kHermiteMatrix[10], kHermiteMatrix[11],
      kHermiteMatrix[12], kHermiteMatrix[13], kHermiteMatrix[14],
      kHermiteMatrix[15]);
}

}  // namespace

std::vector<ignition::math::Vector3d> SplineToBezier(
    const ignition::math::Vector3d& p0, const ignition::math::Vector3d& t0,
    const ignition::math::Vector3d& p1, const ignition::math::Vector3d& t1) {
  // These are the control points arranged by coordinate.
  const ignition::math::Matrix4d hermite_points(
      p0.X(), p0.Y(), p0.Z(), 0.0, t0.X(), t0.Y(), t0.Z(), 0.0, p1.X(), p1.Y(),
      p1.Z(), 0.0, t1.X(), t1.Y(), t1.Z(), 0.0);
  // Given a function F_B(t): ℝ --> ℝ^3 that represents a cubic Bezier curve,
  // and F_H(t) : ℝ --> ℝ^3 that represents a cubic Hermite Spline curve, we can
  // define them like:
  // F_B(t) = [1 t t^2 t^3] * [kBezierMatrix] * [bezier_points]
  // F_H(t) = [1 t t^2 t^3] * [kHermiteMatrix] * [hermite_points]
  // If both functions have the same image: F_B(t) = F_H(t), we can say:
  // [bezier_points] =
  //     [kBezierMatrix]^(-1) * [kHermiteMatrix] * [hermite_points]
  // [hermite_points] =
  //     [kHermiteMatrix]^(-1) * [kBezierMatrix] * [bezier_points]
  const ignition::math::Matrix4d bezier_points =
      BezierBasis().Inverse() * HermiteBasis() * hermite_points;
  std::vector<ignition::math::Vector3d> result;
  result.push_back(ignition::math::Vector3d(
      bezier_points(0, 0), bezier_points(0, 1), bezier_points(0, 2)));
  result.push_back(ignition::math::Vector3d(
      bezier_points(1, 0), bezier_points(1, 1), bezier_points(1, 2)));
  result.push_back(ignition::math::Vector3d(
      bezier_points(2, 0), bezier_points(2, 1), bezier_points(2, 2)));
  result.push_back(ignition::math::Vector3d(
      bezier_points(3, 0), bezier_points(3, 1), bezier_points(3, 2)));
  return result;
}

std::vector<ignition::math::Vector3d> BezierToSpline(
    const ignition::math::Vector3d& p0, const ignition::math::Vector3d& p1,
    const ignition::math::Vector3d& p2, const ignition::math::Vector3d& p3) {
  // These are the control points arranged by coordinate.
  const ignition::math::Matrix4d bezier_points(
      p0.X(), p0.Y(), p0.Z(), 0.0, p1.X(), p1.Y(), p1.Z(), 0.0, p2.X(), p2.Y(),
      p2.Z(), 0.0, p3.X(), p3.Y(), p3.Z(), 0.0);
  // Given a function F_B(t): ℝ --> ℝ^3 that represents a cubic Bezier curve,
  // and F_H(t) : ℝ --> ℝ^3 that represents a cubic Hermite Spline curve, we can
  // define them like:
  // F_B(t) = [1 t t^2 t^3] * [kBezierMatrix] * [bezier_points]
  // F_H(t) = [1 t t^2 t^3] * [kHermiteMatrix] * [hermite_points]
  // If both functions have the same image: F_B(t) = F_H(t), we can say:
  // [bezier_points] =
  //     [kBezierMatrix]^(-1) * [kHermiteMatrix] * [hermite_points]
  // [hermite_points] =
  //     [kHermiteMatrix]^(-1) * [kBezierMatrix] * [bezier_points]
  const ignition::math::Matrix4d hermite_points =
      HermiteBasis().Inverse() * BezierBasis() * bezier_points;
  std::vector<ignition::math::Vector3d> result;
  result.push_back(ignition::math::Vector3d(
      hermite_points(0, 0), hermite_points(0, 1), hermite_points(0, 2)));
  result.push_back(ignition::math::Vector3d(
      hermite_points(1, 0), hermite_points(1, 1), hermite_points(1, 2)));
  result.push_back(ignition::math::Vector3d(
      hermite_points(2, 0), hermite_points(2, 1), hermite_points(2, 2)));
  result.push_back(ignition::math::Vector3d(
      hermite_points(3, 0), hermite_points(3, 1), hermite_points(3, 2)));
  return result;
}

std::vector<ignition::math::Vector3d> MakeBezierCurveMonotonic(
    const std::vector<ignition::math::Vector3d>& control_points,
    double scale = 1.0) {
  DRAKE_THROW_UNLESS(control_points.size() == 4);
  DRAKE_THROW_UNLESS(scale <= 1.0);
  DRAKE_THROW_UNLESS(scale >= 0.0);
  // Sets a constant to compare cross product results near zero.
  const double kAlmostZero = 1e-3;
  // References to control points for better code readability.
  const ignition::math::Vector3d& p0 = control_points[0];
  const ignition::math::Vector3d& p1 = control_points[1];
  const ignition::math::Vector3d& p2 = control_points[2];
  const ignition::math::Vector3d& p3 = control_points[3];

  // Computes normalized tangents at the beginning and the ending.
  const ignition::math::Vector3d norm_t0 = (p1 - p0).Normalize();
  const ignition::math::Vector3d norm_t3 = (p3 - p2).Normalize();
  // Computes a vector that joins curve's endings.
  const ignition::math::Vector3d r = p3 - p0;
  // Computes cross products to determine if the lines will intersect or not.
  const ignition::math::Vector3d norm_t3_x_r = norm_t3.Cross(r);
  const ignition::math::Vector3d norm_t3_x_norm_t0 = norm_t3.Cross(norm_t0);

  std::vector<ignition::math::Vector3d> result;
  if (norm_t3_x_r.Length() < kAlmostZero ||
      norm_t3_x_norm_t0.Length() < kAlmostZero) {
    // Lines are not coplanar or are parallel so they will not intersect. As
    // RNDF does not support any other z coordinate different from 0.0, it is
    // assumed that lines are coplanar and parallel. Consequently, control
    // points for the resulting Bezier curve are adjusted to match a smooth
    // S-shaped transition.
    const double r_dot_norm_t0 = norm_t0.Dot(r);
    result.push_back(p0);
    result.push_back(p0 + (0.5 * r_dot_norm_t0) * norm_t0);
    result.push_back(p3 + (-0.5 * r_dot_norm_t0) * norm_t3);
    result.push_back(p3);
    return result;
  }
  // Computes the vector to move from p0 towards the control point.
  const ignition::math::Vector3d l =
      (norm_t3_x_r.Length() / norm_t3_x_norm_t0.Length()) * norm_t0;
  // Checks the addition or subtraction of l to get the critical point.
  double projection = norm_t3_x_r.Dot(norm_t3_x_norm_t0);
  ignition::math::Vector3d critical_point;
  if (projection >= kAlmostZero) {
    critical_point = p0 + l;
  } else {
    critical_point = p0 - l;
  }

  // TODO(hidmic) Dynamically optimize scales to meet geometrical constraints
  // in the non convex curve case. Current adjustment factor is the result of
  // a manual optimization to achieve the best results with the models we've
  // dealt with so far (i.e. OSM-based RNDFs).
  const double kNonConvexCurveFactor = 0.1;

  // Computes if the resulting Bezier curve will preserve convexity.
  const ignition::math::Vector3d diff_to_p0 = (critical_point - p0);
  const ignition::math::Vector3d diff_to_p3 = (p3 - critical_point);
  // In case the curve should not preserve convexity (first two conditions),
  // intermediate control points will be set to produce the desired geometry.
  if (diff_to_p3.Normalized().Dot(norm_t3) < kAlmostZero) {
    result.push_back(p0);
    result.push_back(
        p0 + kNonConvexCurveFactor * scale * (critical_point - p0));
    result.push_back(
        p3 + (-kNonConvexCurveFactor) * scale * (critical_point - p3));
    result.push_back(p3);
  } else if (diff_to_p0.Normalized().Dot(norm_t0) < kAlmostZero) {
    result.push_back(p0);
    result.push_back(
        p0 + (-kNonConvexCurveFactor) * scale * (critical_point - p0));
    result.push_back(
        p3 + kNonConvexCurveFactor * scale * (critical_point - p3));
    result.push_back(p3);
  } else {
    result.push_back(p0);
    result.push_back(p0 + scale * (critical_point - p0));
    result.push_back(p3 + scale * (critical_point - p3));
    result.push_back(p3);
  }
  return result;
}

std::unique_ptr<ignition::math::Spline> CreatePChipBasedSpline(
    const std::vector<ignition::math::Vector3d>& positions) {
  // Checks if the size of the vector is OK.
  DRAKE_THROW_UNLESS(positions.size() > 2);
  // Adds the knots and breaks.
  std::vector<double> breaks;
  std::vector<MatrixX<double>> knots(positions.size(),
                                     MatrixX<double>::Zero(3, 1));
  for (int i = 0; i < static_cast<int>(positions.size()); i++) {
    knots[i] << positions[i].X(), positions[i].Y(), 0.0;
    if (i == 0) {
      breaks.push_back(0.0);
    } else {
      const double length = (positions[i] - positions[i - 1]).Length();
      // TODO(@agalabachicar) We don't support yet duplicate waypoints in the
      // same lane which are continuous and share the same location.
      // See issue https://bitbucket.org/ekumen/terminus-simulation/issues/171.
      DRAKE_THROW_UNLESS(length > 0.0);
      breaks.push_back(length + breaks.back());
    }
  }
  // Creates the PChip curve and its first derivative so interpolation of
  // tangents at the knots is possible.
  const PiecewisePolynomial<double> polynomial =
      PiecewisePolynomial<double>::Pchip(breaks, knots, false);
  const PiecewisePolynomial<double> derivated_polynomial =
      polynomial.derivative(1);
  // Creates a spline and adds PChip's tangents.
  std::unique_ptr<ignition::math::Spline> spline =
      std::make_unique<ignition::math::Spline>();
  spline->AutoCalculate(true);
  for (int i = 0; i < static_cast<int>(positions.size()); i++) {
    const Vector3<double> tangent = derivated_polynomial.value(breaks[i]);
    spline->AddPoint(positions[i],
                     ignition::math::Vector3d(tangent.x(), tangent.y(), 0.0));
  }
  return spline;
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake

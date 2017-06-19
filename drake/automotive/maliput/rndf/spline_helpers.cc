#include "drake/automotive/maliput/rndf/spline_helpers.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

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

}  // namespace rndf
}  // namespace maliput
}  // namespace drake

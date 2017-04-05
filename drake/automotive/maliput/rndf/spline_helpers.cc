#include <algorithm>

#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

static const int kFunctionPartitionTreeDegree = 10;

static const int kFunctionPartitionTreeMaxDepth = 10;

InverseFunctionInterpolator::InverseFunctionInterpolator(
    const std::function<double(double)> _function, const double _xmin,
    const double _xmax, const double _error_boundary)
    : function_(_function), error_boundary_(_error_boundary) {
  // Make sure the error boundary is attainable.
  DRAKE_THROW_UNLESS(_error_boundary > 0);
  // Instantiate the partition tree root
  this->partition_tree_ = std::make_unique<FunctionGraphPartition>();
  this->partition_tree_->segment.xmin = _xmin;
  this->partition_tree_->segment.xmax = _xmax;
  this->partition_tree_->segment.ymin = _function(_xmin);
  this->partition_tree_->segment.ymax = _function(_xmax);
  this->partition_tree_degree_ = kFunctionPartitionTreeDegree;
  this->partition_tree_max_depth_ = kFunctionPartitionTreeMaxDepth;
}

double
InverseFunctionInterpolator::InterpolateMthDerivative(const int _mth,
                                                      const double _y) const {
  // Make sure that the derivative order is a positive integer or zero.
  DRAKE_THROW_UNLESS(_mth >= 0);
  // Make sure that y is not above x(y) image interval upper bound.
  DRAKE_THROW_UNLESS((_y - this->partition_tree_->segment.ymax) <
                     this->error_boundary_);
  // Make sure that y is not below x(y) image interval low bound.
  DRAKE_THROW_UNLESS((this->partition_tree_->segment.ymin - _y) <
                     this->error_boundary_);

  // Zero out any derivative of higher order than
  // 1, as this is a piecewise linear interpolant.
  if (_mth > 1)
    return 0.0;

  // Get partition tree root.
  FunctionGraphPartition *root = this->partition_tree_.get();

  // Clamp interpolation y
  double iy = std::min(std::max(_y, root->segment.ymin), root->segment.ymax);

  // Traverse down the partition tree until the required
  // error is attained.
  int depth = 0;
  while (true) {
    if (root->partitions.empty()) {
      // Visiting a a leaf node.

      // Linearly interpolate x for the given y.
      double dx = root->segment.xmax - root->segment.xmin;
      double dy = root->segment.ymax - root->segment.ymin;
      double ix = root->segment.xmin + (dx / dy) * (iy - root->segment.ymin);

      // Check if the interpolation suffices error bound requirements.
      if (std::abs(this->function_(ix) - iy) < this->error_boundary_) {
        if (_mth == 1) {
          // Return derivative dx/dy at y.
          return dx / dy;
        }
        // Return interpolated x(y).
        return ix;
      }

      // Safety check before building next tree level.
      DRAKE_THROW_UNLESS(depth <= this->partition_tree_max_depth_);

      // Build sub-partitions for this partition.
      double segmentdx = dx / this->partition_tree_degree_;
      for (int i = 0; i < this->partition_tree_degree_; ++i) {
        double segmentxmin = root->segment.xmin + i * segmentdx;
        double segmentxmax = segmentxmin + segmentdx;

        auto subp = std::make_unique<FunctionGraphPartition>();
        DRAKE_DEMAND(this->partition_tree_ != nullptr);
        subp->segment.xmin = segmentxmin;
        subp->segment.xmax = segmentxmax;
        subp->segment.ymin = this->function_(segmentxmin);
        subp->segment.ymax = this->function_(segmentxmax);
        root->partitions.push_back(std::move(subp));
      }
    }
    // Visiting an inner node.

    // Search subpartition that contains y.
    auto it = std::find_if(
        root->partitions.begin(), root->partitions.end(),
        [&iy](const std::unique_ptr<FunctionGraphPartition> &subp) {
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
    std::unique_ptr<ignition::math::Spline> _spline,
    const double _error_boundary)
    : q_t_(std::move(_spline)) {
  DRAKE_ASSERT(this->q_t_ != nullptr);
  // Instantiate an inverse function interpolator
  // for the given spline arc length function.
  t_s_ = std::make_unique<InverseFunctionInterpolator>([this](double t) {
    return this->q_t_->ArcLength(t);
  }, 0.0, 1.0, _error_boundary);
}

ignition::math::Vector3d
ArcLengthParameterizedSpline::InterpolateMthDerivative(const int _mth,
                                                       const double _s) const {
  if (_mth > 3) {
    // M > 3 => p = 0 (as this is a cubic interpolator)
    return ignition::math::Vector3d(0.0, 0.0, 0.0);
  }
  double t_s = this->t_s_->InterpolateMthDerivative(0, _s);
  if (_mth < 1) {
    // M = 0 => P(s) = Q(t(s))
    return this->q_t_->InterpolateMthDerivative(0, t_s);
  }
  double t_prime_s = this->t_s_->InterpolateMthDerivative(1, _s);
  ignition::math::Vector3d q_prime_t =
      this->q_t_->InterpolateMthDerivative(1, t_s);

  if (_mth < 2) {
    // M = 1 => P'(s) = Q'(t(s)) * t'(s)
    return q_prime_t * t_prime_s;
  }
  double t_prime_s_2 = t_prime_s * t_prime_s;
  double t_prime2_s = this->t_s_->InterpolateMthDerivative(2, _s);
  ignition::math::Vector3d q_prime2_t =
      this->q_t_->InterpolateMthDerivative(2, t_s);
  if (_mth < 3) {
    // M = 2 => P''(s) = Q''(t(s)) * t'(s)^2 + Q'(t(s)) * t''(s)
    return q_prime2_t * t_prime_s_2 + q_prime_t * t_prime2_s;
  }
  double t_prime_s_3 = t_prime_s_2 * t_prime_s;
  double t_prime3_s = this->t_s_->InterpolateMthDerivative(3, _s);
  ignition::math::Vector3d q_prime3_t =
      this->q_t_->InterpolateMthDerivative(3, t_s);
  // M = 3 => P'''(s) = Q'''(t(s)) * t'(s)^3
  ///                   + 3 * Q''(t(s)) * t'(s) * t''(s)
  ///                   + Q'(t(s)) * t'''(s)
  return (q_prime3_t * t_prime_s_3 + 3 * q_prime2_t * t_prime_s * t_prime2_s +
          q_prime_t * t_prime3_s);
}

double ArcLengthParameterizedSpline::FindClosestPointTo(
    const ignition::math::Vector3d &point, const double step) const {
  double closest_s = 0.0;
  double min_distance = std::numeric_limits<double>::max();
  double distance, t_s;
  for (double s = 0.0; s < q_t_->ArcLength(); s += step) {
    t_s = t_s_->InterpolateMthDerivative(0, s);
    distance = (q_t_->InterpolateMthDerivative(0, t_s) - point).Length();
    if (distance < min_distance) {
      min_distance = distance;
      closest_s = s;
    }
  }
  t_s = this->t_s_->InterpolateMthDerivative(0, q_t_->ArcLength());
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

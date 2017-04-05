#pragma once

#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// A segment of a single-variable
/// function graph.
struct FunctionGraphSegment {
  double xmin;  ///< Lower x bound.
  double xmax;  ///< Upper x bound.
  double ymin;  ///< Lower y bound.
  double ymax;  ///< Upper y bound.
};

/// A partition of a single-variable
/// function graph, as a segment plus
/// a set of sub partitions.
struct FunctionGraphPartition {
  /// Segment covered.
  FunctionGraphSegment segment;
  /// Sub-partitions covered.
  std::vector<std::unique_ptr<
    FunctionGraphPartition>> partitions;
};

/// An interpolator for arbitrary inverse functions.
/// Helpful for arc length parameterization with
/// ignition::math::Splines.
class InverseFunctionInterpolator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseFunctionInterpolator)

  /// Constructor that takes the function, its domain
  /// interval and an error boundary for the interpolation.
  /// @param[in] _function arbitrary function for which to
  /// interpolate an inverse
  /// @param[in] _xmin domain interval low bound.
  /// @param[in] _xmax domain interval upper bound.
  /// @param[in] _error_boundary a positive constrain on the
  /// maximum error allowed when approximating the inverse
  /// function.
  /// @throws whenever arguments constrains are not
  /// satisfied.
  explicit InverseFunctionInterpolator(
      const std::function<double(double)> _function,
      const double _xmin, const double _xmax,
      const double _error_boundary);

  /// Interpolates @f$ x(y) @f$, that is, the inverse of the given
  /// function.
  /// @param[in] _mth a positive plus 0 integer describing
  /// the order of the inverse function derivative to interpolate.
  /// @param[in] _y to interpolate at, constrained by the function image.
  /// @return interpolated mth derivative @f$ x^m(y) @f$ .
  /// @throws whenever arguments constrains are not
  /// satisfied.
  double InterpolateMthDerivative(const int _mth,
                                  const double _y) const;

 private:
  const std::function<double(double)> function_;  ///< Base function.
  double error_boundary_;  ///< Error boundary for interpolation.
  std::unique_ptr<
    FunctionGraphPartition> partition_tree_;  ///< Function partition tree.
  int partition_tree_degree_;  ///< Function partition tree degree.
  int partition_tree_max_depth_;  ///< Function partition tree max depth.
};

/// An extension for ignition::math::Splines that reparameterizes
/// them by arc length.
class ArcLengthParameterizedSpline {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcLengthParameterizedSpline)

  /// Constructor that takes a spline and an error bound
  /// for arc length parameterization approximations.
  /// @param[in] _spline to interpolate for.
  /// @param[in] _error_boundary a positive constrain on the
  /// maximum error allowed when approximating the arc length
  /// parameterization.
  /// @throws whenever arguments constrains are not satisfied.
  explicit ArcLengthParameterizedSpline(
    std::unique_ptr<ignition::math::Spline> _spline,
    const double _error_boundary);

  /// Interpolates @f$ Q(s) @f$, that is, the spline parameterized
  /// by arc length.
  /// @param[in] _mth a positive plus 0 integer describing
  /// the order of the function derivative to interpolate.
  /// @param[in] _s arc length to interpolate at, constrained.
  /// by the curve dimensions [0, arc_length].
  /// @return the mth derivative Q^m(s)
  /// @throws whenever arguments constrains are not satisfied.
  ignition::math::Vector3d InterpolateMthDerivative(
      const int _mth, const double _s) const;

  /// Returns a mutable pointer to the underlying spline
  inline ignition::math::Spline* BaseSpline() {
    return this->q_t_.get();
  }

  double FindClosestPointTo(
    const ignition::math::Vector3d &point, const double step) const;

 private:
  std::unique_ptr<ignition::math::Spline> q_t_;  ///< Parameterized curve Q(t).
  std::unique_ptr<
    InverseFunctionInterpolator> t_s_;  ///< Inverse arc length function t(s).
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake

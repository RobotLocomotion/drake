#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// A linear interpolator for arbitrary inverse functions.
/// Helpful for path-length parameterization with ignition::math::Splines.
/// Given a function F and its domain D and codomain CD, this class gives a
/// linear interpolant of the inverse function IF that maps that codomain CD to
/// that domain D.
class InverseFunctionInterpolator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseFunctionInterpolator)

  /// Constructor that takes the function, its domain
  /// interval and an error boundary for the interpolation.
  /// @p function will be used to get the boundary values of the codomain of it
  /// and given the range [ @p xmin, @p xmax] and state the image range used in
  /// this interpolator. Future calls to
  /// InverseFunctionInterpolator::InterpolateMthDerivative should provide
  /// values of y within that image.
  ///
  /// @param[in] function arbitrary continuous function for which to
  /// interpolate an inverse. It should be monotonically increasing with x.
  /// @param[in] xmin @p function 's domain interval low bound.
  /// @param[in] xmax @p function 's domain interval upper bound.
  /// @param[in] error_boundary a positive constraint on the maximum error
  /// allowed when approximating the inverse function.
  /// @throws when @p error_boundary is not positive.
  /// @throws when evaluating @p function with @p xmin and @p xmax throws.
  explicit InverseFunctionInterpolator(
      const std::function<double(double)> function, double xmin, double xmax,
      double error_boundary);

  /// Interpolates @f$ x^{(mth)}(y) @f$, that is, the inverse of the given
  /// function.
  /// @param[in] mth a non-negative integer describing
  /// the order of the inverse function derivative to interpolate. Any value
  /// bigger than 0 will make the function return 0.0 as this is a linear
  /// interpolant.
  /// @param[in] y the range value to interpolate at, constrained by the direct
  /// function image, which is inside the codomain of the direct function.
  /// @return interpolated mth derivative @f$ x^{(m)}(y) @f$ .
  /// @throws when @p mth is a negative integer.
  /// @throws when @p y is bigger than the image of the supplied function
  /// at the constructor for xmax by the error_bound.
  /// @throws when @p y is smaller than the image of the supplied function
  /// at the constructor for xmin by the error_bound.
  double InterpolateMthDerivative(int mth, double y);

 private:
  // A segment of a single-variable function graph.
  struct FunctionGraphSegment {
    double xmin;  //< Lower x bound.
    double xmax;  //< Upper x bound.
    double ymin;  //< Lower y bound.
    double ymax;  //< Upper y bound.
  };

  // A partition of a single-variable function graph, as a segment plus a set
  // of sub partitions.
  struct FunctionGraphPartition {
    // Segment covered.
    FunctionGraphSegment segment;
    // Sub-partitions covered.
    std::vector<std::unique_ptr<FunctionGraphPartition>> partitions;
  };

  const std::function<double(double)> function_;  //< Base function.
  const double error_boundary_ {};  //< Error boundary for interpolation.
  const int partition_tree_degree_ {};  //< Function partition tree degree.
  const int
      partition_tree_max_depth_ {};  //< Function partition tree max depth.
  std::unique_ptr<FunctionGraphPartition>
      partition_tree_;  //< Function partition tree.
};

/// An extension for ignition::math::Splines that reparameterizes
/// them by path length.
class ArcLengthParameterizedSpline {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcLengthParameterizedSpline)

  /// Constructor that takes a spline and an error bound
  /// for path length parameterization approximations.
  /// @param[in] spline to interpolate for.
  /// @param[in] error_boundary a positive constraint on the
  /// maximum error allowed when approximating the path length
  /// parameterization.
  /// @throws when @p spline is nullptr.
  /// @throws when @p error_boundary is not a positive number.
  explicit ArcLengthParameterizedSpline(
      std::unique_ptr<ignition::math::Spline> spline, double error_boundary);

  /// Interpolates @f$ Q(s) @f$, that is, the spline parameterized
  /// by path length.
  /// @param[in] mth a non-negative integer describing
  /// the order of the function derivative to interpolate. Since cubic
  /// interpolation is done, any derivative order greater than 3 will be zero.
  /// @param[in] s path length to interpolate at, constrained
  /// by the curve dimensions [0, path_length].
  /// @return the mth derivative Q^{(m)}(s) .
  /// @throws if @p mth is a negative integer.
  ignition::math::Vector3d InterpolateMthDerivative(int mth, double s);

  /// @return a mutable pointer to the underlying spline.
  inline ignition::math::Spline* BaseSpline() { return this->q_t_.get(); }

  /// Finds the closest point on the spline to a point in space.
  ///
  /// It will iterate over the range of s of the spline which is 0 to 1 in terms
  /// of ignition::math::Spline's parameter at constant path length @p step
  /// increments. On each iteration we get the distance and save the path
  /// length coordinate that gives the minimum distance.
  ///
  /// @param point is the point in space from which we want to get the s path
  /// length value whose image of the direct function (the spline) within 0 and
  /// the total length of the direct function.
  /// @param step is the path length increment to iterate over the complete
  /// spline path length.
  /// @return the path length value that applied to the
  /// ignition::math::Spline, which was provided at construction time, has a
  /// minimum distance to @p point.
  double FindClosestPointTo(const ignition::math::Vector3d& point,
                            double step) const;

 private:
  std::unique_ptr<ignition::math::Spline> q_t_;  //< Parameterized curve Q(t).
  std::unique_ptr<InverseFunctionInterpolator>
      F_ts_;  //< Inverse path length function t(s).
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake

#pragma once

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace automotive {

/// RoadPath converts a sequence of Maliput Lanes into a PiecewisePolynomial for
/// the purpose of generating a path for a car to follow.  The path is created
/// from the start of a user-specified initial lane and direction of travel, and
/// proceeds in that direction until either the end of the road is reached
/// (there exist no ongoing lanes), or the given number of specified breaks have
/// been traversed.  The resulting path is a cubic spline that matches the `r =
/// 0` coordinate of the lanes at each specified break point.  The resulting
/// piecewise curve is C2-continuous throughout with zero first and second
/// derivatives at the start and end of the path.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double is supported.
template <typename T>
class RoadPath {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadPath)

  /// Constructs a single RoadPath from a sequence of Maliput lanes based on the
  /// following parameters:
  /// @param initial_lane_direction contains the initial LaneDirection.  This
  /// must be a valid @p road Lane.
  /// @param step_size is the size of each step (in the `s`-direction) between
  /// knot points.
  /// @param num_breaks are the number of breaks at which the knot points will
  /// be evaluated.
  RoadPath(const LaneDirection& initial_lane_direction, const T& step_size,
           int num_breaks);
  ~RoadPath();

  const PiecewisePolynomial<T>& get_path() const;

  /// Computes the closest `s`-position on the path to an arbitrary point in
  /// the world frame of the provided Maliput Lanes.  (Not yet implemented)
  // TODO(jadecastro): Implement this.
  const T GetClosestPathPosition(const Vector3<T>& geo_position,
                                 const T& s_guess) const;

 private:
  // Traverse the road, starting from an initial LaneDirection, and build a
  // cubic spline PiecewisePolynomial until 1) a given a number of segments has
  // been traversed, or 2) the end of the road has been reached.
  //
  // If a BranchPoint is encountered in which there is more than one ongoing
  // lane, the zero-index lane is always selected.
  // TODO(jadecastro): Use Lane::GetDefaultBranch() to decide the ongoing Lane.
  const PiecewisePolynomial<T> MakePiecewisePolynomial(
      const LaneDirection& initial_lane_direction, const T& step_size,
      int num_breaks) const;

  PiecewisePolynomial<T> path_;  // The path representing the mid-curve of the
                                 // road.
  PiecewisePolynomial<T> path_prime_;         // First derivative of path_.
  PiecewisePolynomial<T> path_double_prime_;  // Second derivative of path_.
};

}  // namespace automotive
}  // namespace drake

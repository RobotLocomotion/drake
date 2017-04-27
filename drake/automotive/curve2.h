#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/// Curve2 represents a path through two-dimensional Cartesian space. Given a
/// list of waypoints, it traces a path between them.
///
/// TODO(jwnimmer-tri) We will soon trace the path using a spline, but
/// for now it's easiest to just interpolate straight segments, as a
/// starting point.  Callers should not yet rely on <em>how</em> we
/// are traversing between the waypoints.
///
template <typename T>
class Curve2 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Curve2)

  /// A two-dimensional Cartesian point that is alignment-safe.
  typedef Eigen::Matrix<T, 2, 1, Eigen::DontAlign> Point2;

  /// Constructor that traces through the given @p waypoints in order.
  /// Throws an error if @p waypoints.size() == 1.
  explicit Curve2(const std::vector<Point2>& waypoints)
      : waypoints_(waypoints), path_length_(GetLength(waypoints_)) {
    // TODO(jwnimmer-tri) We should reject duplicate adjacent
    // waypoints (derivative problems); this will probably come for
    // free as part of the spline refactoring.
  }

  /// @return the length of this curve (the total distance traced).
  T path_length() const { return path_length_; }

  /// A result type for the GetPosition method.
  struct PositionResult {
    Point2 position = Point2{Point2::Zero()};
    Point2 position_dot = Point2{Point2::Zero()};
  };

  /// Returns the Curve's @p PositionResult::position at @p path_distance,
  /// as well as its first derivative @p PositionResult::position_dot with
  /// respect to @p path_distance.
  ///
  /// The @p path_distance is clipped to the ends of the curve:
  /// - A negative @p path_distance is interpreted as a @p path_distance
  ///   of zero.
  /// - A @p path_distance that exceeds the @p path_length() of the curve
  ///   is interpreted as a @p path_distance equal to the @p path_length().
  ///
  /// The @p position_dot derivative, when evaluated exactly at a waypoint,
  /// will be congruent with the direction of one of the (max two) segments
  /// that neighbor the waypoint.  (At the first and last waypoints, there
  /// is only one neighboring segment.)  TODO(jwnimmer-tri) This will no
  /// longer be true once this class uses a spline.
  PositionResult GetPosition(const T& path_distance) const {
    // TODO(jwnimmer-tri) This implementation is slow (linear search)
    // and incorrect (discontinuous; not a spline).  But it will do
    // for now, until we get a 2d spline code in C++.

    PositionResult result;

    // We need at least one segment.  If not, we're just zero.
    if (waypoints_.size() < 2) {
      return result;
    }

    // Iterate over the segments, up through the requested path_distance.
    T remaining_distance = std::max(T{}, path_distance);
    for (auto point0 = waypoints_.begin(), point1 = point0 + 1;
         point1 != waypoints_.end();  // BR
         point0 = point1++) {
      const Point2 relative_step{*point1 - *point0};
      const T length = relative_step.norm();
      if (remaining_distance <= length) {
        auto fraction = remaining_distance / length;
        result.position = Point2{*point0 + fraction * relative_step};
        result.position_dot.head(2) = relative_step / length;
        return result;
      }
      remaining_distance -= length;
    }

    // Oops, we ran out of waypoints; return the final one.
    // The position_dot is congruent with the final segment.
    {
      result.position = waypoints_.back();
      Point2 ultimate = waypoints_.back();
      Point2 penultimate = waypoints_.at(waypoints_.size() - 2);
      const Point2 relative_step{ultimate - penultimate};
      const T length = relative_step.norm();
      result.position_dot.head(2) = relative_step / length;
    }

    return result;
  }

 private:
  // TODO(jwnimmer-tri) Make sure this uses the spline length, not
  // sum-segment-length, once this class uses a spline.
  static T GetLength(const std::vector<Point2>& waypoints) {
    T result{0.0};
    if (waypoints.empty()) {
      return result;
    }
    if (waypoints.size() == 1) {
      throw std::invalid_argument{"single waypoint"};
    }
    for (auto point0 = waypoints.begin(), point1 = point0 + 1;
         point1 != waypoints.end();  // BR
         point0 = point1++) {
      const Point2 relative_step{*point1 - *point0};
      const T length = relative_step.norm();
      result += length;
    }
    return result;
  }

  std::vector<Point2> waypoints_;
  T path_length_;
};

}  // namespace automotive
}  // namespace drake

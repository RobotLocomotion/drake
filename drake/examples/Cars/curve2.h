#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace drake {

/// Curve2 -- a path through two-dimensional cartesian space.
///
/// Given a list of waypoints, traces a path between them.
///
/// TODO(jwnimmer-tri) We will soon trace the path using a spline, but
/// for now it's easiest to just interpolate straight segments, as a
/// starting point.  Callers should not yet rely on <em>how</em> we
/// are traversing between the waypoints.
///
template <typename T>
class Curve2 {
 public:
  /// A two-dimensional cartesian point that is alignment-safe.
  typedef Eigen::Matrix<T, 2, 1, Eigen::DontAlign> Point2;

  /// Constructor that traces through the given @p waypoints in order.
  explicit Curve2(const std::vector<Point2>& waypoints)
      : waypoints_(waypoints), path_length_(GetLength(waypoints_)) {
    // TODO(jwnimmer-tri) We should reject duplicate adjacent
    // waypoints (deriviate problems); this will probably come for
    // free as part of the spline refactoring.
  }

  // Copyable.
  Curve2(const Curve2&) = default;
  Curve2& operator=(const Curve2&) = default;

  /// @return the length of this curve (the total distance traced).
  T path_length() const { return path_length_; }

  /// A result type for the GetPosition method.
  struct PositionResult {
    Point2 position{Point2::Zero()};
    Point2 position_dot{Point2::Zero()};
  };

  /// Return the Curve's position at @p path_distance, along with its
  /// first derivative with respect to @p path_distance.  At the first
  /// and last waypoint, the derivative is the one-sided result coming
  /// from the path side (in other words, ignoring that the path
  /// ends).  A negative @p path_distance will return the first
  /// waypoint.  A @p path_distance that exceeds the length of the
  /// curve will return the last waypoint.
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
    result.position = waypoints_.back();
    // The position_dot is not well defined here, but for now at least
    // returning the left-side approach is more useful for calling code.
    {
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
    if (waypoints.size() < 2) {
      return result;
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

  const std::vector<Point2> waypoints_;
  const T path_length_;
};

}  // namespace drake

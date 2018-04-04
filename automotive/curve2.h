#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiffxd_make_coherent.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace automotive {

template <typename T>
using Point2 = Eigen::Matrix<T, 2, 1, Eigen::DontAlign>;
using Point2d = Point2<double>;

/// Contains a waypoint consisting of a 2-D x,y position and, optionally, speed
/// (default zero).
template <typename T>
struct WaypointT {
  /// Default constructor.
  WaypointT() = default;
  /// Fully-parameterized constructor.
  WaypointT(const Point2<T>& pos, const T& s) : position(pos), speed(s) {}
  /// Position-only constructor.
  explicit WaypointT(const Point2<T>& pos) : position(pos) {}

  Point2<T> position{Point2<T>::Zero()};
  T speed{0.};
  // TODO(jadecastro) Add additional fields as necessary.
};

using Waypoint = WaypointT<double>;

/// A result type for the Curve2::CalcPositionResult method.
/// N.B. The Curve2 implementation is only C1-continuous, so we do not provide
/// AutoDiff for speed_dot.
/// TODO(jadecastro) Enable AutoDiff support.
template <typename T>
struct PositionResult : WaypointT<T> {
  /// Default constructor.
  PositionResult() = default;
  /// Fully-parameterized constructor.
  PositionResult(const Point2<T>& pos, const T& s, const Point2d& pos_deriv,
                 double s_dot)
      : WaypointT<T>(pos, s), position_deriv(pos_deriv), speed_dot(s_dot) {}

  Point2d position_deriv{Point2d::Zero()};
  double speed_dot{0.};
};

/// Curve2 represents a path through two-dimensional Cartesian space. Given a
/// list of waypoints, it linearly interpolates between them.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// TODO(jwnimmer-tri) We will soon trace the path using a spline, but
/// for now it's easiest to just interpolate straight segments, as a
/// starting point.  Callers should not yet rely on <em>how</em> we
/// are traversing between the waypoints.
/// TODO(jadecastro) Consider re-using (wrapping?) PiecewisePolynomial for
/// splines.
template <typename T>
class Curve2 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Curve2)

  explicit Curve2(const std::vector<Waypoint>& waypoints)
      : waypoints_(waypoints), path_length_(GetLengthAndCheck(waypoints)) {
    // TODO(jwnimmer-tri) We should reject duplicate adjacent
    // waypoints (derivative problems); this will probably come for
    // free as part of the spline refactoring.
  }

  /// @return the waypoints associated with this curve.
  const std::vector<Waypoint>& waypoints() const { return waypoints_; }

  /// @return the length of this curve (the total distance traced).
  double path_length() const { return path_length_; }

  /// Returns Curve2's @p PositionResult::position at @p path_distance, as well
  /// as its first derivative @p PositionResult::position_deriv with respect to
  /// @p path_distance and acceleration @p PositionResult::speed_dot.
  ///
  /// The @p path_distance is clipped to the ends of the curve:
  /// - A negative @p path_distance is interpreted as a @p path_distance
  ///   of zero.
  /// - A @p path_distance that exceeds the @p path_length() of the curve
  ///   is interpreted as a @p path_distance equal to the @p path_length().
  ///
  /// The @p position_deriv, when evaluated exactly at a waypoint, will be
  /// congruent with the direction of one of the (max two) segments that
  /// neighbor the waypoint.  (At the first and last waypoints, there is only
  /// one neighboring segment.)
  ///
  /// The @p speed_dot, time derivative of speed, is piecewise-constant for the
  /// linear interpolation scheme used herein.  With the exception of the final
  /// waypoint, where speed_dot is zero, speed_dot at waypoint i is the same as
  /// that for segment {i, i+1}.
  ///
  /// TODO(jwnimmer-tri) This will no longer be true once this class uses a
  /// spline.
  PositionResult<T> CalcPositionResult(const T& path_distance) const {
    using std::max;

    // TODO(jwnimmer-tri) This implementation is slow (linear search)
    // and incorrect (discontinuous; not a spline).  But it will do
    // for now, until we get a 2d spline code in C++.

    PositionResult<T> result;

    // We need at least one segment.  If not, ignore the waypoints and return
    // the default PositionResult.
    if (waypoints_.size() < 2) {
      return result;
    }

    // Iterate over the segments, up through the requested path_distance,
    // linearly interpolating the result (position and speed) between waypoints.
    T remaining_distance = max(T{0.0}, path_distance);
    for (auto point0 = waypoints_.begin(), point1 = point0 + 1;
         point1 != waypoints_.end();  // BR
         point0 = point1++) {
      const Point2d relative_position_step{point1->position - point0->position};
      const T length{relative_position_step.norm()};
      if (remaining_distance <= length) {
        return MakeResult(path_distance, remaining_distance, *point0, *point1,
                          ExtractDoubleOrThrow(length));
      }
      remaining_distance -= length;
    }

    // Oops, we ran out of waypoints; return the final one.
    // The position_dot is congruent with the final segment.
    {
      const Waypoint ultimate(waypoints_.back());
      const Waypoint penultimate(waypoints_.at(waypoints_.size() - 2));
      const Point2d relative_position_step{ultimate.position -
                                           penultimate.position};
      const double length = relative_position_step.norm();
      result = MakeResult(
          path_distance, remaining_distance, penultimate, ultimate, length);
    }

    return result;
  }

 private:
  // Computes the path length with respect to x-y Waypoint::position, checking
  // for positivity and consistency of the Waypoint::speed field.
  //
  // TODO(jwnimmer-tri) Make sure this uses the spline length, not
  // sum-segment-length, once this class uses a spline.
  static double GetLengthAndCheck(const std::vector<Waypoint>& waypoints) {
    double result{0.0};
    if (waypoints.empty()) {
      return result;
    }
    if (waypoints.size() == 1) {
      throw std::invalid_argument{"single waypoint"};
    }
    for (const auto& point : waypoints) {
      DRAKE_THROW_UNLESS(point.speed >= 0.);
    }
    for (auto point0 = waypoints.begin(), point1 = point0 + 1;
         point1 != waypoints.end();  // BR
         point0 = point1++) {
      const Point2d relative_step{point1->position - point0->position};
      const double length = relative_step.norm();
      result += length;
    }
    return result;
  }

  // Helper that returns PositionResult given the current waypoint and relative
  // position to go from the penultimate waypoint.
  const PositionResult<T> MakeResult(const T& path_distance,
                                     const T& remaining_distance,
                                     const Waypoint& point0,
                                     const Waypoint& point1,
                                     double length) const {
    const Point2d relative_position_step{point1.position - point0.position};
    const double relative_speed_step{point1.speed - point0.speed};
    const double average_speed_step{point1.speed + point0.speed};
    double speed_dot{relative_speed_step * average_speed_step / 2 / length};
    const Point2d position_deriv =
        relative_position_step / ExtractDoubleOrThrow(length);
    if (remaining_distance > path_length_) {
      Point2<T> position{point1.position};
      T speed{point1.speed};
      MakeWaypointCoherent(path_distance, &position, &speed);
      return {position, speed, position_deriv, speed_dot};
    }
    auto fraction = remaining_distance / length;
    Point2<T> position =
        point0.position + fraction * Point2<T>{relative_position_step};
    T speed = point0.speed + fraction * T{relative_speed_step};
    MakeWaypointCoherent(path_distance, &position, &speed);
    return {position, speed, position_deriv, speed_dot};
  }

  static void MakeWaypointCoherent(const T& donor, Point2<T>* pos, T* speed) {
    autodiffxd_make_coherent(donor, &(*pos)(0));
    autodiffxd_make_coherent(donor, &(*pos)(1));
    autodiffxd_make_coherent(donor, speed);
  }

  std::vector<Waypoint> waypoints_;
  double path_length_{};
};

}  // namespace automotive
}  // namespace drake

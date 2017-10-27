#pragma once

#include "drake/automotive/gen/pure_pursuit_params.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// PurePursuit computes the required steering angle to achieve a goal point on
/// an continuous planar path.  The path represents as the set of `r = 0`
/// positions along a Maliput lane, and a goal point is selected as a
/// pre-defined lookahead distance along the path in the intended direction of
/// travel.  The algorithm outputs the steering angle required to guide the
/// vehicle toward the goal point based on its current position in global
/// coordinates.
///
/// See [1] and the corresponding .cc file for details on the algorithm.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// [1] Coulter, R. Implementation of the Pure Pursuit Path Tracking
///     Algorithm. Carnegie Mellon University, Pittsburgh, Pennsylvania, Jan
///     1990.
template <typename T>
class PurePursuit {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuit)
  PurePursuit() = delete;

  /// Evaluates the required steering angle in radians using the pure-pursuit
  /// method.  Assumes zero elevation and superelevation.
  ///
  /// @param pp_params contains the `lookahead_distance`, the distance along the
  /// path based on the closest position on the path to the vehicle.
  ///
  /// @param car_params contains the `wheelbase` of the vehicle.
  ///
  /// @param lane_direction is a LaneDirection containing a reference lane and
  /// the direction of travel along the positive-s coordinate.
  ///
  /// @param pose is the PoseVector for the ego vehicle.
  // TODO(jadecastro): Infer the direction of travel rather than require it.
  static T Evaluate(const PurePursuitParams<T>& pp_params,
                    const SimpleCarParams<T>& car_params,
                    const LaneDirection& lane_direction,
                    const systems::rendering::PoseVector<T>& pose);

  /// Computes the goal point at a distance `s_lookahead` from the closest
  /// position on the curve in the intended direction of travel, and `with_s`
  /// and `pose` are the direction of travel and PoseVector for the ego vehicle.
  static const maliput::api::GeoPosition ComputeGoalPoint(
      const T& s_lookahead, const LaneDirection& lane_direction,
      const systems::rendering::PoseVector<T>& pose);
};

}  // namespace automotive
}  // namespace drake

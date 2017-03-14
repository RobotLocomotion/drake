#pragma once

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/// IdmPlanner (Intelligent Driver Model Planner) is a simple model governing
/// longitudinal accelerations of a vehicle in single-lane traffic [1, 2].  It
/// is derived based on qualitative observations of actual driving behavior and
/// captures objectives such as keeping a safe distance behind a lead vehicle,
/// maintaining a desired speed, and accelerating and decelerating within
/// comfortable limits.
///
/// The IDM equation produces accelerations that realize smooth transitions
/// between the following three modes:
///  - Free-road behavior: when the distance to the leading car is large, the
///    IDM regulates acceleration to match the desired speed `v_0`.
///  - Fast-closing-speed behavior: when the target distance decreases, an
///    interaction term compensates for the velocity difference, while keeping
///    deceleration comfortable according to parameter `b`.
///  - Small-distance behavior: within small net distances to the lead vehicle,
///    comfort is ignored in favor of increasing this distance to `s_0`.
///
/// See the corresponding .cc file for details about the IDM equation.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// [1] Martin Treiber and Arne Kesting. Traffic Flow Dynamics, Data, Models,
///     and Simulation. Springer, 2013.
///
/// [2] https://en.wikipedia.org/wiki/Intelligent_driver_model.
///
/// @ingroup automotive_systems
template <typename T>
class IdmPlanner {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmPlanner)
  IdmPlanner() = delete;

  /// Evaluates the IDM equation for the chosen planner parameters @p params,
  /// given the current velocity @p ego_velocity, distance to the lead car @p
  /// target_distance, and the closing velocity @p target_distance_dot.  The
  /// returned value is a longitudinal acceleration.
  static const T Evaluate(const IdmPlannerParameters<T>& params,
                          const T& ego_velocity, const T& target_distance,
                          const T& target_distance_dot);

  /// Sets defaults for all parameters.
  static void SetDefaultParameters(IdmPlannerParameters<T>* params);
};

}  // namespace automotive
}  // namespace drake

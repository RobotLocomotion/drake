#pragma once

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/// IdmPlanner -- an IDM (Intelligent Driver Model) planner.  The IDM is a
/// simple model governing longitudinal accelerations of a vehicle in
/// single-lane traffic [1, 2].  It is derived based on qualitative obvervations
/// of actual driving behavior such as keeping a safe distance away from a lead
/// vehicle, maintaining a desired speed, and accelerating and decelerating
/// within comfortable limits.
///
/// The model captures three basic driving modes:
///  - Free-road behavior: when the target distance to the leading car is large,
///    the IDM regulates acceleration to match the desired speed @p v_0.
///  - Behavior at fast closing speeds: when the target distance decreases, an
///    interaction term compensates for the velocity difference, while keeping
///    deceleration comfortable according to parameter @p b.
///  - Behavior at small target distances: within small distances, comfort is
///    ignored in favor of increasing the target distance to @p s_0.
///
/// IDM creates a smooth transitioning between these modes.
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
/// [2] https://en.wikipedia.org/wiki/Intelligent_driver_model.
///
/// @ingroup automotive_systems
template <typename T>
class IdmPlanner {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmPlanner)
  IdmPlanner() = delete;

  /// Evaluates the IDM equation for the chosen planner parameters @param
  /// params, given the current velocity @param ego_velocity, distance to the
  /// lead car @param target_distance, and the closing velocity @param
  /// target_distance_dot.
  static const T Evaluate(const IdmPlannerParameters<T>& params,
                          const T& ego_velocity, const T& target_distance,
                          const T& target_distance_dot);

  /// Sets defaults for all parameters.
  static void SetDefaultParameters(IdmPlannerParameters<T>* params);
};

}  // namespace automotive
}  // namespace drake

#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/automotive/gen/idm_planner_parameters.h"

namespace drake {
namespace automotive {

/// IdmPlanner -- an IDM (Intelligent Driver Model) planner.
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class IdmPlanner {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmPlanner)
  IdmPlanner() = delete;

  static T Evaluate(
      const IdmPlannerParameters<T>& params,
      const T& ego_velocity,
      const T& target_distance,
      const T& target_distance_dot);

  /// Sets defaults for all parameters.  Most callers will want to replace
  /// v_ref with a new value, because the default is 1.0 m/s.
  static void SetDefaultParameters(IdmPlannerParameters<T>* params);

};

}  // namespace automotive
}  // namespace drake

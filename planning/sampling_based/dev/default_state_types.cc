#include "drake/planning/sampling_based/dev/default_state_types.h"

#include <type_traits>

namespace drake {
namespace planning {
template <typename StateType>
ControlPlanningState<StateType>::ControlPlanningState() {
  // Ensure fixed-size state types are zeroed in the default constructor.
  if constexpr (std::is_same_v<Eigen::Vector2d, StateType> ||
                std::is_same_v<Eigen::Vector3d, StateType>) {
    state_ = StateType::Zero();
  }
}

}  // namespace planning
}  // namespace drake

// Manually define the supported instantiations of ControlPlanningState.
template class ::drake::planning::ControlPlanningState<::Eigen::Vector2d>;
template class ::drake::planning::ControlPlanningState<::Eigen::Vector3d>;
template class ::drake::planning::ControlPlanningState<
    ::drake::math::RigidTransformd>; /* NOLINT(whitespace/line_length) */
template class ::drake::planning::ControlPlanningState<::Eigen::VectorXd>;

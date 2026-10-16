#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace planning {
/// Basic planning state for control planning, incorporating state and control.
/// State value is one of {Vector2d, Vector3d, RigidTransformd, VectorXd} and
/// control is always VectorXd.
template <typename StateType>
class ControlPlanningState {
 public:
  /// Provides all copy/move/assign operations.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ControlPlanningState);

  /// Constructs with the provided state and an empty control.
  explicit ControlPlanningState(const StateType& state) : state_(state) {}

  /// Constructs with the provided state and control.
  ControlPlanningState(const StateType& state, const Eigen::VectorXd& control)
      : state_(state), control_{control} {}

  /// Default constructor.
  /// For fixed-size vector state types (e.g. Vector2d and Vector3d), the
  /// default state is zeroed.
  ControlPlanningState();

  const StateType& state() const { return state_; }

  const Eigen::VectorXd& control() const { return control_; }

  StateType& mutable_state() { return state_; }

  Eigen::VectorXd& mutable_control() { return control_; }

  void SetState(const StateType& state) { state_ = state; }

  void SetControl(const Eigen::VectorXd& control) { control_ = control; }

  bool has_control() const { return control_.size() > 0; }

 private:
  StateType state_;
  Eigen::VectorXd control_;
};

/// Eight potential planning state types are supported.
///
/// Four types for purely kinematic planning:
/// - Eigen::Vector2d for R^2 planning (primarily examples and test cases)
/// - Eigen::Vector3d for SE(2) planning
/// - RigidTransformd for SE(3) planning
/// - Eigen::VectorXd for vector-valued planning
///
/// Four equivalent types for kinodynamic "control" planning, which combine a
/// state type with a vector-valued control:
/// - ControlPlanningState<Eigen::Vector2d> for R^2 control planning
/// - ControlPlanningState<Eigen::Vector3d> for SE(2) control planning
/// - ControlPlanningState<RigidTransformd> for SE(3) control planning
/// - ControlPlanningState<Eigen::VectorXd> for vector-valued control planning

/// Defines template instantiations for Anzu's default planning state types.
/// This should only be used in .cc files, never in .h files.
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(  \
    SomeType)                                                                \
  template SomeType<::Eigen::Vector2d>;                                      \
  template SomeType<::Eigen::Vector3d>;                                      \
  template SomeType<::drake::math::RigidTransformd>;                         \
  template SomeType<::Eigen::VectorXd>;                                      \
  template SomeType<                                                         \
      ::drake::planning::ControlPlanningState<::Eigen::Vector2d>>;           \
  template SomeType<                                                         \
      ::drake::planning::ControlPlanningState<::Eigen::Vector3d>>;           \
  template SomeType<::drake::planning::ControlPlanningState<                 \
      ::drake::math::RigidTransformd>>; /* NOLINT(whitespace/line_length) */ \
  template SomeType<::drake::planning::ControlPlanningState<::Eigen::VectorXd>>;

/// Declares that template instantiations exist for Anzu's default planning
/// state types.
/// This should only be used in .h files, never in .cc files.
#define DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES( \
    SomeType)                                                                \
  extern template SomeType<::Eigen::Vector2d>;                               \
  extern template SomeType<::Eigen::Vector3d>;                               \
  extern template SomeType<::drake::math::RigidTransformd>;                  \
  extern template SomeType<::Eigen::VectorXd>;                               \
  extern template SomeType<::drake::planning::ControlPlanningState<          \
      ::Eigen::Vector2d>>; /* NOLINT(whitespace/line_length) */              \
  extern template SomeType<::drake::planning::ControlPlanningState<          \
      ::Eigen::Vector3d>>; /* NOLINT(whitespace/line_length) */              \
  extern template SomeType<::drake::planning::ControlPlanningState<          \
      ::drake::math::RigidTransformd>>; /* NOLINT(whitespace/line_length) */ \
  extern template SomeType<::drake::planning::ControlPlanningState<          \
      ::Eigen::VectorXd>>; /* NOLINT(whitespace/line_length) */

}  // namespace planning
}  // namespace drake

// Manually declare the supported instantiations of ControlPlanningState.
extern template class ::drake::planning::ControlPlanningState<
    ::Eigen::Vector2d>;
extern template class ::drake::planning::ControlPlanningState<
    ::Eigen::Vector3d>;
extern template class ::drake::planning::ControlPlanningState<
    ::drake::math::RigidTransformd>; /* NOLINT(whitespace/line_length) */
extern template class ::drake::planning::ControlPlanningState<
    ::Eigen::VectorXd>;

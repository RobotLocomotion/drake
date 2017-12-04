#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_alias_groups.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

/**
 * A base class that outputs QpInput for the qp inverse dynamics controller.
 * This class must be extended to implement custom behaviors. It is designed as
 * a discrete time system. QpInput is stored inside the AbstractState, and
 * updated in DoCalcUnrestrictedUpdate(). Custom plan types are also stored and
 * updated in the same way.
 */
class PlanEvalBaseSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanEvalBaseSystem)

  /**
   * Constructor.
   * @param robot Pointer to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Control time step
   */
  PlanEvalBaseSystem(const RigidBodyTree<double>* robot,
                     const std::string& alias_groups_file_name,
                     const std::string& param_file_name, double dt);

  /**
   * Calls DoExtendedCalcUnrestrictedUpdate().
   */
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const final {
    DoExtendedCalcUnrestrictedUpdate(context, state);
  }

  /**
   * Returns input port for RobotKinematicState.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_kinematic_state() const {
    return get_input_port(input_port_index_kinematic_state_);
  }

  /**
   * Returns output port for QpInput.
   */
  inline const systems::OutputPort<double>& get_output_port_qp_input() const {
    return get_output_port(output_port_index_qp_input_);
  }

  /**
   * Returns the size of the abstract state.
   */
  int get_num_abstract_states() const {
    return 1 + get_num_extended_abstract_states();
  }

  /// @name Accessors
  /// @{
  const RigidBodyTree<double>& get_robot() const { return robot_; }

  double get_control_dt() const { return control_dt_; }

  const RigidBodyTreeAliasGroups<double>& get_alias_groups() const {
    return alias_groups_;
  }

  const systems::controllers::qp_inverse_dynamics::ParamSet& get_paramset()
      const {
    return paramset_;
  }
  /// @}

 protected:
  /**
   * Returns the size of extended abstract state.
   */
  virtual int get_num_extended_abstract_states() const = 0;

  /**
   * Derived classes need to implement this for custom behaviors.
   */
  virtual void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const = 0;

  /**
   * Returns a mutable reference of Type in @p state at @p index
   */
  template <typename Type>
  Type& get_mutable_abstract_value(systems::State<double>* state,
                                   int index) const {
    return state->get_mutable_abstract_state()
        .get_mutable_value(index)
        .GetMutableValue<Type>();
  }

  /**
   * Returns a mutable reference to QpInput in @p state.
   */
  systems::controllers::qp_inverse_dynamics::QpInput& get_mutable_qp_input(
      systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        .get_mutable_value(abs_state_index_qp_input_)
        .GetMutableValue<systems::controllers::qp_inverse_dynamics::QpInput>();
  }

 private:
  // Copies QpInput from abstract state to the corresponding output port.
  void CopyOutQpInput(
      const systems::Context<double>& context,
      systems::controllers::qp_inverse_dynamics::QpInput* output) const;

  const RigidBodyTree<double>& robot_;
  const double control_dt_{};

  RigidBodyTreeAliasGroups<double> alias_groups_;
  systems::controllers::qp_inverse_dynamics::ParamSet paramset_;

  int input_port_index_kinematic_state_{};
  int output_port_index_qp_input_{};

  int abs_state_index_qp_input_{-1};
};

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake

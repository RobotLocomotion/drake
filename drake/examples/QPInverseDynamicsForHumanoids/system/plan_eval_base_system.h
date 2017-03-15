#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

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
   * @param robot Reference to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Control time step
   */
  PlanEvalBaseSystem(const RigidBodyTree<double>& robot,
                     const std::string& alias_groups_file_name,
                     const std::string& param_file_name, double dt);

  /**
   * Allocates abstract value types for output @p descriptor. This function
   * allocates QpInput when @p matches the port for QpInput, and calls
   * ExtendedAllocateOutputAbstract() to allocate all the other derived class'
   * custom output types.
   */
  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const final;

  /**
   * Copies QpInput from abstract state to the corresponding output port. Then
   * calls DoExtendedCalcOutput() to handle all the other derived class' custom
   * outputs.
   */
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const final;

  /**
   * Calls DoExtendedCalcUnrestrictedUpdate().
   */
  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const final {
    DoExtendedCalcUnrestrictedUpdate(context, state);
  }

  /**
   * Calls ExtendedAllocateAbstractState() first to allocate derived class'
   * custom abstract states, then appends a QpInput to the end.
   * @return The combined AbstractState.
   */
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState() const final;

  /**
   * Returns input port for HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * Returns output port for QpInput.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_qp_input()
      const {
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

  const param_parsers::RigidBodyTreeAliasGroups<double>& get_alias_groups()
      const {
    return alias_groups_;
  }

  const param_parsers::ParamSet& get_paramset() const { return paramset_; }
  /// @}

 protected:
  /**
   * Returns the size of extended abstract state.
   */
  virtual int get_num_extended_abstract_states() const = 0;

  /**
   * Derived classes need to implement this to computed custom outputs.
   */
  virtual void DoExtendedCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const = 0;

  /**
   * Derived classes need to implement this to allocate custom outputs.
   */
  virtual std::unique_ptr<systems::AbstractValue>
  ExtendedAllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const = 0;

  /**
   * Derived classes need to implement this for custom behaviors.
   */
  virtual void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const = 0;

  /**
   * Derived classes need to implement this to allocate custom abstract states.
   */
  virtual std::vector<std::unique_ptr<systems::AbstractValue>>
  ExtendedAllocateAbstractState() const = 0;

  /**
   * Returns a mutable reference of Type in @p state at @p index
   */
  template <typename Type>
  Type& get_mutable_abstract_value(systems::State<double>* state,
                                   int index) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_value(index)
        .GetMutableValue<Type>();
  }

  /**
   * Returns a mutable reference to QpInput in @p state.
   */
  QpInput& get_mutable_qp_input(systems::State<double>* state) const {
    int size = state->get_mutable_abstract_state()->size();
    return state->get_mutable_abstract_state()
        ->get_mutable_value(size - 1)
        .GetMutableValue<QpInput>();
  }

 private:
  const RigidBodyTree<double>& robot_;
  const double control_dt_{};

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups_;
  param_parsers::ParamSet paramset_;

  int input_port_index_humanoid_status_{};
  int output_port_index_qp_input_{};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

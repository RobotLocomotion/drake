#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A base block that generates qp input for the qp inverse dynamics controller.
 * This class must be extended to implement the desired behaviors. Due to its
 * discrete time nature, control is computed in DoCalcUnrestrictedUpdate(),
 * and the result is stored in its AbstractState. DoCalcOutput() merely copies
 * the latest result from its AbstractState and sends it through the output
 * port. Context's time must be properly maintained. The internal states of
 * the plan are also stored in its AbstractState, and can be modified in
 * DoCalcUnrestrictedUpdate().
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

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override {
    throw std::runtime_error(
        "Subclass need to implememt DoCalcUnrestrictedUpdate.");
  }

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override {
    throw std::runtime_error(
        "Subclass need to implememt AllocateOutputAbstract.");
    return std::make_unique<systems::AbstractState>();
  }

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

  /// @name Accessors
  /// @{
  const RigidBodyTree<double>& get_robot() const { return robot_; }

  double get_control_dt() const { return kControlDt; }

  const param_parsers::RigidBodyTreeAliasGroups<double>& get_alias_groups()
      const {
    return alias_groups_;
  }

  const param_parsers::ParamSet& get_paramset() const { return paramset_; }
  /// @}

 protected:
  // Returns a mutable reference of custom plan in abstract state.
  template <typename PlanType>
  PlanType& get_mutable_plan(systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_abstract_state(kAbsStateIdxPlan)
        .GetMutableValue<PlanType>();
  }

  // Returns a mutable reference of QpInput in abstract state.
  QpInput& get_mutable_qp_input(systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_abstract_state(kAbsStateIdxQpInput)
        .GetMutableValue<QpInput>();
  }

  int get_input_port_index_humanoid_status() const {
    return input_port_index_humanoid_status_;
  }
  int get_output_port_index_qp_input() const {
    return output_port_index_qp_input_;
  }
  int get_abstract_state_index_qp_input() const { return kAbsStateIdxQpInput; }
  int get_abstract_state_index_plan() const { return kAbsStateIdxPlan; }

 private:
  const RigidBodyTree<double>& robot_;
  const double kControlDt;

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups_;
  param_parsers::ParamSet paramset_;

  int input_port_index_humanoid_status_{0};
  int output_port_index_qp_input_{0};
  const int kAbsStateIdxQpInput{0};
  const int kAbsStateIdxPlan{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

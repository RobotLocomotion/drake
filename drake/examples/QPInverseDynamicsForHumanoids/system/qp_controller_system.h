#pragma once

#include <memory>

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A wrapper around qp inverse dynamics controller.
 *
 * Input: HumanoidStatus
 * Input: QpInput
 * Output: QpOutput
 */
class QPControllerSystem : public systems::LeafSystem<double> {
 public:
  QPControllerSystem(const RigidBodyTree<double>& robot, double dt);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the input: QpInput.
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_qp_input()
      const {
    return get_input_port(input_port_index_qp_input_);
  }

  /**
   * @return Port for the output: QpOutput.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_qp_output() const {
    return get_output_port(output_port_index_qp_output_);
  }

 private:
  QpOutput& get_mutable_qp_output(systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_abstract_state(abstract_state_qp_output_index_)
        .GetMutableValue<QpOutput>();
  }

  const RigidBodyTree<double>& robot_;
  const double control_dt_{0.002};
  const int abstract_state_qp_output_index_{0};

  // TODO(siyuan.feng): This is a bad temporary hack to the const constraint for
  // CalcOutput. It is because qp controller needs to allocate mutable workspace
  // (MathematicalProgram, temporary matrices for doing math, etc),
  // and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  int input_port_index_humanoid_status_{0};
  int input_port_index_qp_input_{0};
  int output_port_index_qp_output_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

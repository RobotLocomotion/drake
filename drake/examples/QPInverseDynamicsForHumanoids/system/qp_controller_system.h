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
  explicit QPControllerSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

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
  const RigidBodyTree<double>& robot_;

  // TODO(siyuan.feng): This is a bad temporary hack to the const constraint for
  // CalcOutput. It is because qp controller needs to allocate mutable workspace
  // (MathematicalProgram, temporary matrices for doing math, etc),
  // and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  int input_port_index_humanoid_status_;
  int input_port_index_qp_input_;
  int output_port_index_qp_output_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

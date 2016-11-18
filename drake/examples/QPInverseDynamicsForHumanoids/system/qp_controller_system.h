#pragma once

#include <iostream>

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A wrapper around qp inverse dynamics controller.
 *
 * Input: HumanoidStatus
 * Input: QPInput
 * Output: QPOutput
 */
class QPControllerSystem : public systems::LeafSystem<double> {
 public:
  explicit QPControllerSystem(const RigidBodyTree<double>& robot)
      : robot_(robot) {
    input_port_index_humanoid_status_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    input_port_index_qp_input_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    output_port_index_qp_input_ =
        DeclareAbstractOutputPort(systems::kInheritedSampling).get_index();

    DRAKE_ASSERT(this->get_num_input_ports() == 2);
    DRAKE_ASSERT(this->get_num_output_ports() == 1);

    set_name("qp_controller");
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // Inputs:
    const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    const lcmt_qp_input* qp_input_msg =
        EvalInputValue<lcmt_qp_input>(context, input_port_index_qp_input_);

    QPInput qp_input(robot_);
    DecodeQPInput(robot_, *qp_input_msg, &qp_input);

    // Output:
    QPOutput& qp_output = output->GetMutableData(output_port_index_qp_input_)
                              ->GetMutableValue<QPOutput>();

    if (qp_controller_.Control(*rs, qp_input, &qp_output) < 0) {
      std::cout << rs->position().transpose() << std::endl;
      std::cout << rs->velocity().transpose() << std::endl;
      std::cout << qp_input << std::endl;
      throw std::runtime_error("System2QP: QP canot solve\n");
    }
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    QPOutput out(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<QPOutput>(out)));
    return std::move(output);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the input: QPInput.
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_input() const {
    return get_input_port(input_port_index_qp_input_);
  }

  /**
   * @return Port for the output: QPOutput.
   */
  inline const SystemPortDescriptor<double>& get_output_port_qp_output() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  // TODO(siyuan.feng): This is a bad temporary hack to the const constraint for
  // EvalOutput. It is because qp controller needs to allocate mutable workspace
  // (MathematicalProgram, temporary matrices for doing math, etc),
  // and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  int input_port_index_humanoid_status_;
  int input_port_index_qp_input_;
  int output_port_index_qp_input_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

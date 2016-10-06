#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace systems {
namespace qp_inverse_dynamics {

// TODO(siyuan.feng@tri.global): This is a bad temporary hack to the const
// constraint for EvalOutput. It is because qp controller needs to allocate
// mutable workspace (MathematicalProgram, temporary matrices for doing math,
// etc), and I want to avoid allocating these repeatedly.
static example::qp_inverse_dynamics::QPController qp_controller__;

class System2QP : public LeafSystem<double> {
 public:
  /**
   * A system2 wrapper around the qp inverse dynamics controller.
   * Input: humanoid status, qp input
   * Output: qp outout
   */
  explicit System2QP(const RigidBodyTree& robot) : robot_(robot) {
    input_port_num_humanoid_status_ =
        DeclareAbstractInputPort(kInheritedSampling).get_index();
    input_port_num_qp_input_ =
        DeclareAbstractInputPort(kInheritedSampling).get_index();
    output_port_num_qp_input_ =
        DeclareAbstractOutputPort(kInheritedSampling).get_index();

    DRAKE_ASSERT(this->get_num_input_ports() == 2);
    DRAKE_ASSERT(this->get_num_output_ports() == 1);

    set_name("qp_controller");
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // get robot status
    const example::qp_inverse_dynamics::HumanoidStatus* rs =
        EvalInputValue<example::qp_inverse_dynamics::HumanoidStatus>(
            context, input_port_num_humanoid_status_);

    // get qp input
    const example::qp_inverse_dynamics::QPInput* qp_input =
        EvalInputValue<example::qp_inverse_dynamics::QPInput>(
            context, input_port_num_qp_input_);

    example::qp_inverse_dynamics::QPOutput& qp_output =
        output->GetMutableData(output_port_num_qp_input_)
            ->GetMutableValue<example::qp_inverse_dynamics::QPOutput>();

    if (qp_controller__.Control(*rs, *qp_input, &qp_output) < 0) {
      throw std::runtime_error("System2QP: QP canot solve\n");
    }
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    example::qp_inverse_dynamics::QPOutput out(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<example::qp_inverse_dynamics::QPOutput>(out)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

  /**
   * return input port number that corresponds to: humanoid status
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_num_humanoid_status_);
  }

  /**
   * return input port number that corresponds to: qp input
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_input() const {
    return get_input_port(input_port_num_qp_input_);
  }

  /**
   * return output port number that corresponds to: qp output
   */
  inline const SystemPortDescriptor<double>& get_output_port_qp_output() const {
    return get_output_port(output_port_num_qp_input_);
  }

 private:
  const RigidBodyTree& robot_;

  int input_port_num_humanoid_status_;
  int input_port_num_qp_input_;
  int output_port_num_qp_input_;
};

}  // end namespace qp_inverse_dynamics
}  // end namespace example
}  // end namespace drake

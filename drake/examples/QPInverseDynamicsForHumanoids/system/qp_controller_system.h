#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class QPControllerSystem : public systems::LeafSystem<double> {
 public:
  /**
   * Input: humanoid status, qp input
   * Output: qp outout
   */
  explicit QPControllerSystem(const RigidBodyTree& robot) : robot_(robot) {
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
    // Get robot status.
    const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // Get qp input.
    const QPInput* qp_input =
        EvalInputValue<QPInput>(context, input_port_index_qp_input_);

    QPOutput& qp_output = output->GetMutableData(output_port_index_qp_input_)
                              ->GetMutableValue<QPOutput>();

    if (qp_controller_.Control(*rs, *qp_input, &qp_output) < 0) {
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
   * @return the input port number that corresponds to: humanoid status.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return the input port number that corresponds to: qp input.
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_input() const {
    return get_input_port(input_port_index_qp_input_);
  }

  /**
   * @return the output port number that corresponds to: qp output.
   */
  inline const SystemPortDescriptor<double>& get_output_port_qp_output() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree& robot_;

  // TODO(siyuan.feng@tri.global): This is a bad temporary hack to the const
  // constraint for EvalOutput. It is because qp controller needs to allocate
  // mutable workspace (MathematicalProgram, temporary matrices for doing math,
  // etc), and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  int input_port_index_humanoid_status_;
  int input_port_index_qp_input_;
  int output_port_index_qp_input_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

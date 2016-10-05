#pragma once

#include "drake/systems/framework/leaf_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace systems {

static example::qp_inverse_dynamics::QPController qp_controller__;

class System2QP : public LeafSystem<double> {
 public:
  explicit System2QP(const RigidBodyTree &robot) :
    robot_(robot) {
    // make IO ports
    // state input
    this->DeclareAbstractInputPort(kInheritedSampling);

    // qp input
    this->DeclareAbstractInputPort(kInheritedSampling);

    // qp output
    this->DeclareAbstractOutputPort(kInheritedSampling);

    set_name("qp_controller");
  }

  void EvalOutput(const Context<double> &context, SystemOutput<double>* output) const override {
    DRAKE_ASSERT(context.get_num_input_ports() == 2);
    DRAKE_ASSERT(output->get_num_ports() == 1);

    // get robot status
    const example::qp_inverse_dynamics::HumanoidStatus *rs = EvalInputValue<example::qp_inverse_dynamics::HumanoidStatus>(context, 0);

    // get qp input
    const example::qp_inverse_dynamics::QPInput *qp_input = EvalInputValue<example::qp_inverse_dynamics::QPInput>(context, 1);;

    example::qp_inverse_dynamics::QPOutput& qp_output = output->GetMutableData(0)->GetMutableValue<example::qp_inverse_dynamics::QPOutput>();

    if (qp_controller__.Control(*rs, *qp_input, &qp_output) < 0) {
      throw std::runtime_error("System2QP: QP canot solve\n");
    }
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(const Context<double>& context) const {
    std::unique_ptr<LeafSystemOutput<double>> output(new LeafSystemOutput<double>);
    example::qp_inverse_dynamics::QPOutput out(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<example::qp_inverse_dynamics::QPOutput>(out)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

 private:
  const RigidBodyTree &robot_;
};

}
}

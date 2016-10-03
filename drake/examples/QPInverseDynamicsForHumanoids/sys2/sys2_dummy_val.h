#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace systems {

class System2DummyValkyrieSim : public LeafSystem<double> {
 public:
  explicit System2DummyValkyrieSim(const RigidBodyTree &robot)
    : robot_(robot) {

    zero_torque_ = Eigen::VectorXd::Zero(robot_.actuators.size());

    // input: qp_output
    System::DeclareAbstractInputPort(kInheritedSampling);

    // output: huamnoids status
    System::DeclareAbstractOutputPort(kInheritedSampling);

    DRAKE_ASSERT(this->get_num_input_ports() == 1);
    DRAKE_ASSERT(this->get_num_output_ports() == 1);

    DRAKE_ASSERT(this->get_input_port(0).get_data_type() == kAbstractValued);
    DRAKE_ASSERT(this->get_output_port(0).get_data_type() == kAbstractValued);

    set_name("dummy val");
  }

  std::unique_ptr<ContinuousState<double>> AllocateContinuousState() const override {
    int num_q = robot_.get_num_positions();
    int num_v = robot_.get_num_velocities();
    int num_x = num_q + num_v;
    return std::make_unique<ContinuousState<double>>(std::make_unique<BasicVector<double>>(num_x), num_q, num_v, 0);
  }

  bool has_any_direct_feedthrough() const override { return false; }

  void EvalOutput(const Context<double> &context, SystemOutput<double>* output) const override {
    const ContinuousState<double> &state = *context.get_state().get_continuous_state();
    Eigen::VectorXd q = state.get_generalized_position().CopyToVector();
    Eigen::VectorXd v = state.get_generalized_velocity().CopyToVector();

    // set output
    example::qp_inverse_dynamics::HumanoidStatus& rs = output->GetMutableData(0)->GetMutableValue<example::qp_inverse_dynamics::HumanoidStatus>();
    rs.Update(context.get_time(), q, v, zero_torque_, Eigen::Matrix<double,6,1>::Zero(), Eigen::Matrix<double,6,1>::Zero());
  }

  void EvalTimeDerivatives(const Context<double>& context, ContinuousState<double>* derivatives) const override {
    // get acceleration from qpouput
    const example::qp_inverse_dynamics::QPOutput &qpout = context.GetInputPort(0)->get_abstract_data()->GetValue<example::qp_inverse_dynamics::QPOutput>();
    const Eigen::VectorXd &vd = qpout.vd();

    // get current state
    const ContinuousState<double> &state = *context.get_state().get_continuous_state();
    const VectorBase<double> &state_v = state.get_generalized_velocity();

    VectorBase<double> *new_v = derivatives->get_mutable_generalized_position();
    VectorBase<double> *new_vd = derivatives->get_mutable_generalized_velocity();
    if (new_v->size() != state_v.size() || new_vd->size() != vd.size())
      throw std::runtime_error("time deriv dimension mismatch.");

    for (int i = 0; i < new_v->size(); i++)
      new_v->SetAtIndex(i, state_v.GetAtIndex(i));

    for (int i = 0; i < new_vd->size(); i++)
      new_vd->SetAtIndex(i, vd(i));
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(const Context<double>& context) const {
    std::unique_ptr<LeafSystemOutput<double>> output(new LeafSystemOutput<double>);
    example::qp_inverse_dynamics::HumanoidStatus rs(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<example::qp_inverse_dynamics::HumanoidStatus>(rs)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

  void DoPublish(const Context<double>& context) const override {
    const ContinuousState<double> &state = *context.get_state().get_continuous_state();
    Eigen::VectorXd q = state.get_generalized_position().CopyToVector();
    Eigen::VectorXd v = state.get_generalized_velocity().CopyToVector();
    std::cout << "pos: " << q << std::endl;
    std::cout << "vel: " << v << std::endl;
  }

  void set_initial_state(Context<double> *context) {
    context->set_time(0);
    ContinuousState<double> &state = *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double> *q = state.get_mutable_generalized_position();
    VectorBase<double> *v = state.get_mutable_generalized_velocity();

    if (q->size() != robot_.get_num_positions() || v->size() != robot_.get_num_velocities()) {
      throw std::runtime_error("time deriv dimension mismatch.");
    }

    example::qp_inverse_dynamics::HumanoidStatus rs(robot_);
    q->SetFromVector(rs.GetNominalPosition());
    for (int i = 0; i < v->size(); i++)
      v->SetAtIndex(i, 0.);
  }

 private:
  const RigidBodyTree& robot_;
  Eigen::VectorXd zero_torque_;
};

}
}

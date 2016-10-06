#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace systems {
namespace qp_inverse_dynamics {

class System2DummyValkyrieSim : public LeafSystem<double> {
 public:
  /**
   * A sysmtem2 dummy dynamics simulator for Valkyrie.
   * Time derivative is actually computed in the qp controller.
   * Input: qp output
   * Output: humanoid status
   */
  explicit System2DummyValkyrieSim(const RigidBodyTree& robot) : robot_(robot) {
    input_port_num_qp_output_ =
        DeclareAbstractInputPort(kInheritedSampling).get_index();
    output_port_num_humanoid_status_ =
        DeclareAbstractOutputPort(kInheritedSampling).get_index();

    zero_torque_ = Eigen::VectorXd::Zero(robot_.actuators.size());

    DRAKE_ASSERT(this->get_num_input_ports() == 1);
    DRAKE_ASSERT(this->get_num_output_ports() == 1);

    set_name("dummy val");
  }

  bool has_any_direct_feedthrough() const override { return false; }

  std::unique_ptr<ContinuousState<double>> AllocateContinuousState()
      const override {
    int num_q = robot_.get_num_positions();
    int num_v = robot_.get_num_velocities();
    int num_x = num_q + num_v;
    return std::make_unique<ContinuousState<double>>(
        std::make_unique<BasicVector<double>>(num_x), num_q, num_v, 0);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    example::qp_inverse_dynamics::HumanoidStatus rs(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<example::qp_inverse_dynamics::HumanoidStatus>(rs)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    const ContinuousState<double>& state =
        *context.get_state().get_continuous_state();
    Eigen::VectorXd q = state.get_generalized_position().CopyToVector();
    Eigen::VectorXd v = state.get_generalized_velocity().CopyToVector();

    // set output
    example::qp_inverse_dynamics::HumanoidStatus& rs =
        output->GetMutableData(output_port_num_humanoid_status_)
            ->GetMutableValue<example::qp_inverse_dynamics::HumanoidStatus>();
    rs.Update(context.get_time(), q, v, zero_torque_, Eigen::Vector6d::Zero(),
              Eigen::Vector6d::Zero());
  }

  void EvalTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // get acceleration from qpouput
    const example::qp_inverse_dynamics::QPOutput* qpout =
        EvalInputValue<example::qp_inverse_dynamics::QPOutput>(
            context, input_port_num_qp_output_);
    const Eigen::VectorXd& vd = qpout->vd();

    // get current state
    const ContinuousState<double>& state =
        *context.get_state().get_continuous_state();
    const VectorBase<double>& state_v = state.get_generalized_velocity();

    VectorBase<double>* new_v = derivatives->get_mutable_generalized_position();
    VectorBase<double>* new_vd =
        derivatives->get_mutable_generalized_velocity();
    if (new_v->size() != state_v.size() || new_vd->size() != vd.size())
      throw std::runtime_error("time deriv dimension mismatch.");

    for (int i = 0; i < new_v->size(); i++)
      new_v->SetAtIndex(i, state_v.GetAtIndex(i));

    for (int i = 0; i < new_vd->size(); i++) new_vd->SetAtIndex(i, vd(i));
  }

  /**
   * Setup the initial condition: time = 0, q = Valkyrie's nominal q, and v = 0.
   * @return A humanoid status unique pointer with the same q and v.
   */
  std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus>
  SetInitialCondition(Context<double>* context) {
    context->set_time(0);
    ContinuousState<double>& state =
        *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double>* q = state.get_mutable_generalized_position();
    VectorBase<double>* v = state.get_mutable_generalized_velocity();

    if (q->size() != robot_.get_num_positions() ||
        v->size() != robot_.get_num_velocities()) {
      throw std::runtime_error("time deriv dimension mismatch.");
    }

    std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus> rs(
        new example::qp_inverse_dynamics::HumanoidStatus(robot_));
    q->SetFromVector(rs->GetNominalPosition());
    for (int i = 0; i < v->size(); i++) v->SetAtIndex(i, 0.);

    rs->Update(context->get_time(), q->CopyToVector(), v->CopyToVector(),
               zero_torque_, Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    return std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus>(
        rs.release());
  }

  /**
   * Perturb the position in the context.
   * @param position_name, name for the generalized coordinate.
   * @param perturbation, q.at(\p position_name) += perturbation.
   * @param context, system context
   */
  void PerturbPosition(const std::string& position_name, double perturbation,
                       Context<double>* context) {
    ContinuousState<double>& state =
        *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double>* q = state.get_mutable_generalized_position();

    example::qp_inverse_dynamics::HumanoidStatus rs(robot_);
    int idx = rs.name_to_position_index().at(position_name);
    q->SetAtIndex(idx, q->GetAtIndex(idx) + perturbation);
  }

  /**
   * Perturb the velocity in the context.
   * @param velocity_name, name for the generalized coordinate.
   * @param perturbation, v.at(\p position_name) += perturbation.
   * @param context, system context
   */
  void PerturbVelocity(const std::string& velocity_name, double perturbation,
                       Context<double>* context) {
    ContinuousState<double>& state =
        *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double>* v = state.get_mutable_generalized_velocity();

    example::qp_inverse_dynamics::HumanoidStatus rs(robot_);
    int idx = rs.name_to_velocity_index().at(velocity_name);
    v->SetAtIndex(idx, v->GetAtIndex(idx) + perturbation);
  }

  /**
   * @param context, system context
   * @return A humanoid status pointer from \p context
   */
  std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus>
  GetHumanoidStatusFromContext(const Context<double>& context) {
    const ContinuousState<double>& state =
        *context.get_state().get_continuous_state();
    const VectorBase<double>& q = state.get_generalized_position();
    const VectorBase<double>& v = state.get_generalized_velocity();

    if (q.size() != robot_.get_num_positions() ||
        v.size() != robot_.get_num_velocities()) {
      throw std::runtime_error("time deriv dimension mismatch.");
    }

    std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus> rs(
        new example::qp_inverse_dynamics::HumanoidStatus(robot_));
    rs->Update(context.get_time(), q.CopyToVector(), v.CopyToVector(),
               zero_torque_, Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    return std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus>(
        rs.release());
  }

  /**
   * @return port number that corresponds to the input: qp_output.
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_output() const {
    return get_input_port(input_port_num_qp_output_);
  }

  /**
   * @return port number that corresponds to the output: humanoid status.
   */
  inline const SystemPortDescriptor<double>& get_output_port_humanoid_status()
      const {
    return get_output_port(output_port_num_humanoid_status_);
  }

 private:
  const RigidBodyTree& robot_;

  int input_port_num_qp_output_;
  int output_port_num_humanoid_status_;

  Eigen::VectorXd zero_torque_;
};

}  // end namespace qp_inverse_dynamics
}  // end namespace example
}  // end namespace drake

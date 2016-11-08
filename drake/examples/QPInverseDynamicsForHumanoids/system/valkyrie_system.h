#pragma once

#include <string>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

using systems::Context;
using systems::ContinuousState;
using systems::LeafSystem;
using systems::SystemOutput;
using systems::LeafSystemOutput;
using systems::SystemPortDescriptor;
using systems::BasicVector;
using systems::AbstractValue;
using systems::Value;
using systems::VectorBase;

namespace examples {
namespace qp_inverse_dynamics {

/**
 * A dummy simulator for Valkyrie. This does not perform forward dynamics
 * computation. Instead, it uses the computed acceleration from the qp
 * controller.
 *
 * Input: QPOutput (only using the computed accelerations)
 * Output: bot_core::robot_state_t
 * Output: vector based raw robot state (q, v)
 */
class ValkyrieSystem : public LeafSystem<double> {
 public:
  explicit ValkyrieSystem(const RigidBodyTree<double>& robot) : robot_(robot) {
    input_port_index_qp_output_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    output_port_index_robot_state_msg_ =
        DeclareAbstractOutputPort(systems::kInheritedSampling).get_index();
    output_port_index_raw_state_ =
        DeclareOutputPort(
            systems::kVectorValued,
            robot_.get_num_positions() + robot_.get_num_velocities(),
            systems::kInheritedSampling).get_index();

    zero_torque_ = Eigen::VectorXd::Zero(robot_.actuators.size());

    DRAKE_ASSERT(this->get_num_input_ports() == 1);
    DRAKE_ASSERT(this->get_num_output_ports() == 2);

    set_name("Dummy Valkyrie System");

    joint_names_.resize(robot_.get_num_positions() - kTwistSize);
    for (int i = kTwistSize; i < robot_.get_num_positions(); ++i) {
      joint_names_[i - kTwistSize] = robot_.get_position_name(i);
    }
  }

  // Only the state is used to produce the output.
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
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<bot_core::robot_state_t>(bot_core::robot_state_t())));

    output->get_mutable_ports()->emplace_back(new systems::OutputPort(
        AllocateOutputVector(get_output_port_raw_state())));
    return std::move(output);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    const ContinuousState<double>& state =
        *context.get_state().get_continuous_state();
    Eigen::VectorXd q = state.get_generalized_position().CopyToVector();
    Eigen::VectorXd v = state.get_generalized_velocity().CopyToVector();

    // Outputs:
    bot_core::robot_state_t& msg =
        output->GetMutableData(output_port_index_robot_state_msg_)
            ->GetMutableValue<bot_core::robot_state_t>();
    EncodeRobotStateLcmMsg(joint_names_, context.get_time(), q, v, zero_torque_,
                           Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(),
                           &msg);

    // Set state output.
    BasicVector<double>* output_x =
        output->GetMutableVectorData(output_port_index_raw_state_);
    for (int i = 0; i < robot_.get_num_positions(); ++i) {
      output_x->SetAtIndex(i, q[i]);
    }
    for (int i = 0; i < robot_.get_num_velocities(); ++i) {
      output_x->SetAtIndex(i + robot_.get_num_positions(), v[i]);
    }
  }

  void EvalTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // Get the acceleration from qpouput.
    const QPOutput* qpout =
        EvalInputValue<QPOutput>(context, input_port_index_qp_output_);
    const Eigen::VectorXd& vd = qpout->vd();

    // Get the current state.
    const ContinuousState<double>& state =
        *context.get_state().get_continuous_state();
    const VectorBase<double>& state_v = state.get_generalized_velocity();

    VectorBase<double>* new_v = derivatives->get_mutable_generalized_position();
    VectorBase<double>* new_vd =
        derivatives->get_mutable_generalized_velocity();
    if (new_v->size() != state_v.size() || new_vd->size() != vd.size()) {
      throw std::runtime_error("time deriv dimension mismatch.");
    }

    for (int i = 0; i < new_v->size(); ++i) {
      new_v->SetAtIndex(i, state_v.GetAtIndex(i));
    }

    for (int i = 0; i < new_vd->size(); ++i) {
      new_vd->SetAtIndex(i, vd(i));
    }
  }

  /**
   * Set up the initial condition: time = 0, q = Valkyrie's nominal q,
   * and v = 0.
   * @return A humanoid status unique pointer with the same q and v.
   */
  std::unique_ptr<HumanoidStatus> SetInitialCondition(
      Context<double>* context) {
    context->set_time(0);
    ContinuousState<double>& state =
        *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double>* q = state.get_mutable_generalized_position();
    VectorBase<double>* v = state.get_mutable_generalized_velocity();

    if (q->size() != robot_.get_num_positions() ||
        v->size() != robot_.get_num_velocities()) {
      throw std::runtime_error("time deriv dimension mismatch.");
    }

    std::unique_ptr<HumanoidStatus> rs(new HumanoidStatus(robot_));
    q->SetFromVector(rs->GetNominalPosition());
    for (int i = 0; i < v->size(); ++i) {
      v->SetAtIndex(i, 0.);
    }

    rs->Update(context->get_time(), q->CopyToVector(), v->CopyToVector(),
               zero_torque_, Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    return rs;
  }

  /**
   * Perturb the position in the context.
   * @param position_name, name for the generalized coordinate.
   * @param perturbation, q.at(\p position_name) += perturbation.
   * @param context, system context
   */
  void PerturbPosition(const std::string& position_name, double perturbation,
                       Context<double>* context) const {
    ContinuousState<double>& state =
        *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double>* q = state.get_mutable_generalized_position();

    HumanoidStatus rs(robot_);
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
                       Context<double>* context) const {
    ContinuousState<double>& state =
        *context->get_mutable_state()->get_mutable_continuous_state();
    VectorBase<double>* v = state.get_mutable_generalized_velocity();

    HumanoidStatus rs(robot_);
    int idx = rs.name_to_velocity_index().at(velocity_name);
    v->SetAtIndex(idx, v->GetAtIndex(idx) + perturbation);
  }

  /**
   * @param context, system context
   * @return A humanoid status pointer from \p context
   */
  std::unique_ptr<HumanoidStatus> MakeHumanoidStatusFromContext(
      const Context<double>& context) const {
    const ContinuousState<double>& state =
        *context.get_state().get_continuous_state();
    const VectorBase<double>& q = state.get_generalized_position();
    const VectorBase<double>& v = state.get_generalized_velocity();

    if (q.size() != robot_.get_num_positions() ||
        v.size() != robot_.get_num_velocities()) {
      throw std::runtime_error("time deriv dimension mismatch.");
    }

    std::unique_ptr<HumanoidStatus> rs(new HumanoidStatus(robot_));
    rs->Update(context.get_time(), q.CopyToVector(), v.CopyToVector(),
               zero_torque_, Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    return rs;
  }

  /**
   * @return Port for the input: QPOutput.
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_output() const {
    return get_input_port(input_port_index_qp_output_);
  }

  /**
   * @return Port for the output: bot_core::robot_state_t
   */
  inline const SystemPortDescriptor<double>& get_output_port_robot_state_msg()
      const {
    return get_output_port(output_port_index_robot_state_msg_);
  }

  /**
   * @return Port for the output: vector based state (q, v).
   */
  inline const SystemPortDescriptor<double>& get_output_port_raw_state() const {
    return get_output_port(output_port_index_raw_state_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  // only used for publishing.
  std::vector<std::string> joint_names_;

  int input_port_index_qp_output_;
  int output_port_index_robot_state_msg_;
  int output_port_index_raw_state_;

  Eigen::VectorXd zero_torque_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

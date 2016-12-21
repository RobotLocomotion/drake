#pragma once

#include "bot_core/atlas_command_t.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::SystemPortDescriptor;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::BasicVector;
using systems::Value;

/**
 * A stub for a more complex interface for joint level controllers.
 * The idea is to separate all joint level control from the higher level
 * full state feedback controller, e.g. qp inverse dynamics controller.
 * This can also run at a higher rate than the full state feedback controller.
 *
 * Possible things to be implemented here:
 *  joint level set points, gains, integrators, filters,
 *  disturbance observers, etc.
 *
 * Input: HumanoidStatus
 * Input: QPOutput
 * Output: lcm message bot_core::atlas_command_t in channel "ROBOT_COMMAND"
 */
class JointLevelControllerSystem : public systems::LeafSystem<double> {
 public:
  explicit JointLevelControllerSystem(const RigidBodyTree<double>& robot)
      : robot_(robot) {
    in_port_idx_qp_output_ = DeclareAbstractInputPort().get_index();

    in_port_idx_humanoid_status_ = DeclareAbstractInputPort().get_index();

    out_port_index_atlas_cmd_ = DeclareAbstractOutputPort().get_index();

    // TODO(siyuan.fent): Load gains from some config.
    int act_size = static_cast<int>(robot_.actuators.size());
    k_q_p_ = VectorX<double>::Zero(act_size);
    k_q_i_ = VectorX<double>::Zero(act_size);
    k_qd_p_ = VectorX<double>::Zero(act_size);
    k_f_p_ = VectorX<double>::Zero(act_size);
    ff_qd_ = VectorX<double>::Zero(act_size);
    ff_qd_d_ = VectorX<double>::Zero(act_size);
    // Directly feed torque through without any other feedbacks.
    ff_f_d_ = VectorX<double>::Constant(act_size, 1.);
    ff_const_ = VectorX<double>::Zero(act_size);
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    // Inputs
    const QPOutput* qp_output =
        EvalInputValue<QPOutput>(context, in_port_idx_qp_output_);
    const HumanoidStatus* rs =
        EvalInputValue<HumanoidStatus>(context, in_port_idx_humanoid_status_);

    // Output
    bot_core::atlas_command_t& msg =
        output->GetMutableData(out_port_index_atlas_cmd_)
            ->GetMutableValue<bot_core::atlas_command_t>();

    // Make bot_core::atlas_command_t message.
    msg.utime = static_cast<uint64_t>(rs->time() * 1e6);

    int act_size = static_cast<int>(robot_.actuators.size());
    msg.num_joints = act_size;
    msg.joint_names.resize(msg.num_joints);
    msg.position.resize(msg.num_joints);
    msg.velocity.resize(msg.num_joints);
    msg.effort.resize(msg.num_joints);

    // bot_core::atlas_command_t assumes the joints are in actuator ordering.
    // dof torque = RigidBodyTree.B * actuator torque.
    // Assuming no coupling among joints,
    // actuator torque = RigidBodyTree.B.transpose() * dof torque
    VectorX<double> act_torques =
        robot_.B.transpose() * qp_output->dof_torques();
    if (act_torques.size() != act_size) {
      throw std::runtime_error("torque dimension mismatch.");
    }

    // Set desired position, velocity and torque for all actuators.
    for (int i = 0; i < act_size; ++i) {
      msg.joint_names[i] = robot_.actuators[i].name_;
      msg.position[i] = 0;
      msg.velocity[i] = 0;
      msg.effort[i] = act_torques[i];
    }

    eigenVectorToStdVector(k_q_p_, msg.k_q_p);
    eigenVectorToStdVector(k_q_i_, msg.k_q_i);
    eigenVectorToStdVector(k_qd_p_, msg.k_qd_p);
    eigenVectorToStdVector(k_f_p_, msg.k_f_p);
    eigenVectorToStdVector(ff_qd_, msg.ff_qd);
    eigenVectorToStdVector(ff_qd_d_, msg.ff_qd_d);
    eigenVectorToStdVector(ff_f_d_, msg.ff_f_d);
    eigenVectorToStdVector(ff_const_, msg.ff_const);

    // This is only used for the Virtual Robotics Challenge's gazebo simulator.
    // Should be deprecated by now.
    msg.k_effort.resize(msg.num_joints, 0);
    // TODO(siyuan.feng): I am not sure what this does exactly, most likely for
    // deprecated simulation as well.
    // Consider removing this from atlas_command_t.
    msg.desired_controller_period_ms = 0;
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);

    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<bot_core::atlas_command_t>(bot_core::atlas_command_t())));
    return std::move(output);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(in_port_idx_humanoid_status_);
  }

  /**
   * @return Port for the input: QPOutput.
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_output() const {
    return get_input_port(in_port_idx_qp_output_);
  }

  /**
   * @return Port for the output: bot_core::atlas_command_t message
   */
  inline const SystemPortDescriptor<double>& get_output_port_atlas_command()
      const {
    return get_output_port(out_port_index_atlas_cmd_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int in_port_idx_qp_output_;
  int in_port_idx_humanoid_status_;
  int out_port_index_atlas_cmd_;

  // Joint level gains, these are in actuator order.
  VectorX<double> k_q_p_;
  VectorX<double> k_q_i_;
  VectorX<double> k_qd_p_;
  VectorX<double> k_f_p_;
  VectorX<double> ff_qd_;
  VectorX<double> ff_qd_d_;
  VectorX<double> ff_f_d_;
  VectorX<double> ff_const_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

#pragma once

#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

#include "drake/lcm/drake_lcm_interface.h"
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
  JointLevelControllerSystem(const RigidBodyTree<double>& robot,
                             drake::lcm::DrakeLcmInterface* lcm)
      : robot_(robot), lcm_(lcm) {
    in_port_idx_qp_output_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();

    in_port_idx_humanoid_status_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();

    out_port_index_vd_ =
        DeclareOutputPort(
            systems::kVectorValued,
            robot_.get_num_velocities(),
            systems::kInheritedSampling).get_index();

    // TODO(siyuan.fent): Load gains from some config.
    int act_size = robot_.actuators.size();
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

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // Inputs
    const QPOutput* qp_output =
        EvalInputValue<QPOutput>(context, in_port_idx_qp_output_);
    const HumanoidStatus* rs =
        EvalInputValue<HumanoidStatus>(context, in_port_idx_humanoid_status_);

    // Output
    BasicVector<double>* output_vd =
        output->GetMutableVectorData(out_port_index_vd_);
    for (int i = 0; i < robot_.get_num_velocities(); ++i) {
      output_vd->SetAtIndex(i, qp_output->vd()[i]);
    }

    // Make message.
    robot_cmd_msg_.utime = static_cast<uint64_t>(rs->time() * 1e6);

    int act_size = robot_.actuators.size();
    robot_cmd_msg_.num_joints = act_size;
    robot_cmd_msg_.joint_names.resize(robot_cmd_msg_.num_joints);
    robot_cmd_msg_.position.resize(robot_cmd_msg_.num_joints);
    robot_cmd_msg_.velocity.resize(robot_cmd_msg_.num_joints);
    robot_cmd_msg_.effort.resize(robot_cmd_msg_.num_joints);

    // Compute actuator torques from dof torques.
    // Since dof_torque = B * u, and assuming no coupling between joints,
    // u = B.transpose() * dof_torque.
    VectorX<double> act_torques =
        robot_.B.transpose() * qp_output->dof_torques();

    // Set desired position, velocity and torque for all actuators.
    for (int i = 0; i < act_size; ++i) {
      robot_cmd_msg_.joint_names[i] = robot_.actuators[i].name_;
      robot_cmd_msg_.position[i] = 0;
      robot_cmd_msg_.velocity[i] = 0;
      robot_cmd_msg_.effort[i] = act_torques[i];
    }

    eigenVectorToStdVector(k_q_p_, robot_cmd_msg_.k_q_p);
    eigenVectorToStdVector(k_q_i_, robot_cmd_msg_.k_q_i);
    eigenVectorToStdVector(k_qd_p_, robot_cmd_msg_.k_qd_p);
    eigenVectorToStdVector(k_f_p_, robot_cmd_msg_.k_f_p);
    eigenVectorToStdVector(ff_qd_, robot_cmd_msg_.ff_qd);
    eigenVectorToStdVector(ff_qd_d_, robot_cmd_msg_.ff_qd_d);
    eigenVectorToStdVector(ff_f_d_, robot_cmd_msg_.ff_f_d);
    eigenVectorToStdVector(ff_const_, robot_cmd_msg_.ff_const);

    // This is only used for the Virtural Robotics Challenge's gazebo simulator.
    // Should be deprecated by now.
    robot_cmd_msg_.k_effort.resize(robot_cmd_msg_.num_joints, 0);
    // TODO(siyuan.feng): I am not sure what this does exactly, most likely for
    // deprecated simulation as well.
    // Consider removing this from atlas_command_t.
    robot_cmd_msg_.desired_controller_period_ms = 0;
  }

  void DoPublish(const Context<double>& context) const override {
    // Encode and send the lcm message.
    std::vector<uint8_t> raw_bytes;
    int msg_size = robot_cmd_msg_.getEncodedSize();
    raw_bytes.resize(msg_size);
    robot_cmd_msg_.encode(raw_bytes.data(), 0, msg_size);
    lcm_->Publish("ROBOT_COMMAND", raw_bytes.data(), msg_size);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);

    output->get_mutable_ports()->emplace_back(new systems::OutputPort(
        AllocateOutputVector(get_output_port_vd())));
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
   * @return Port for the output: vector representing the generalized
   * acceleration.
   */
  inline const SystemPortDescriptor<double>& get_output_port_vd() const {
    return get_output_port(out_port_index_vd_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int in_port_idx_qp_output_;
  int in_port_idx_humanoid_status_;
  int out_port_index_vd_;

  // I made this mutable primarily because I want to avoid calling EvalInput
  // twice on in_port_idx_qp_output_, which calls the qp controller twice to
  // solve for the exact same problem.
  // There are three ways to eliminate this:
  // 1. Use a real simulator that does not depend the vd from the inverse
  // dynamics controller.
  // 2. I think sys2 cache will fix this (caching the result from the first
  // EvalInput call).
  // 3. Put vd as part of the atlas_command_t message, but I think that's
  // conceptually more hacky.
  // TODO(siyuan.feng) remove this eventually.
  mutable bot_core::atlas_command_t robot_cmd_msg_;

  // LCM publishing interface
  drake::lcm::DrakeLcmInterface* const lcm_;

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

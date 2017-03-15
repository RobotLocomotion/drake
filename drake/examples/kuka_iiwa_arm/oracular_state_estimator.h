#pragma once

#include <memory>
#include <string>

#include "bot_core/robot_state_t.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * A class that takes state vector and output a bot_core::robot_state_t message.
 * Note that the joint_effort part will be set to zero.
 */
template <typename T>
class OracularStateEstimation : public systems::LeafSystem<T> {
 public:
  /**
   * Constructor for OracularStateEstimation.
   * @param robot, Reference to the RigidBodyTree. The life span of @p robot
   * needs to be longer than this instance. Also note that the generated LCM
   * messages will contain every joint in @p robot.
   * @param base_body, Reference to the base link in @p robot. Can be either
   * floating base or a fixed base. @p base_body must be part of @p robot,
   * and it needs to have a longer life span than this instance.
   */
  OracularStateEstimation(const RigidBodyTree<T>& robot,
                          const RigidBody<T>& base_body)
      : robot_(robot), base_body_(base_body) {
    input_port_index_state_ =
        this->DeclareInputPort(
                systems::kVectorValued,
                robot.get_num_positions() + robot.get_num_velocities())
            .get_index();
    output_port_index_msg_ = this->DeclareAbstractOutputPort().get_index();
  }

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override {
    const systems::BasicVector<T>* state =
        this->EvalVectorInput(context, input_port_index_state_);

    VectorX<T> q = state->get_value().head(robot_.get_num_positions());
    VectorX<T> v = state->get_value().tail(robot_.get_num_velocities());
    KinematicsCache<T> cache = robot_.doKinematics(q, v);

    bot_core::robot_state_t& msg =
        output->template GetMutableData(output_port_index_msg_)
            ->template GetMutableValue<bot_core::robot_state_t>();
    msg.utime = static_cast<int64_t>(context.get_time() * 1e6);

    // Pose and velocity of floating body in the world frame.
    Isometry3<T> base_body_to_world =
        robot_.CalcBodyPoseInWorldFrame(cache, base_body_);
    Vector6<T> base_body_velocity =
        robot_.CalcBodySpatialVelocityInWorldFrame(cache, base_body_);

    EncodePose(base_body_to_world, msg.pose);
    EncodeTwist(base_body_velocity, msg.twist);

    // Encodes joint names, positions, velocities and efforts.
    // Note: the order of the actuators in the rigid body tree determines the
    // order of the joint_name, joint_position, joint_velocity, and
    // joint_effort fields.
    msg.joint_name.resize(robot_.get_num_actuators());
    msg.joint_position.resize(robot_.get_num_actuators());
    msg.joint_velocity.resize(robot_.get_num_actuators());
    msg.joint_effort.resize(robot_.get_num_actuators());
    msg.num_joints = static_cast<int16_t>(msg.joint_name.size());
    int i = 0;
    for (const auto& actuator : robot_.actuators) {
      const auto& body = *actuator.body_;

      // To match usage of robot_state_t throughout OpenHumanoids code, set
      // joint_names field to position coordinate names.
      // We are iterating over actuators, so won't have a problem with the
      // floating base here.
      int position_index = body.get_position_start_index();
      int velocity_index = body.get_velocity_start_index();
      msg.joint_name[i] = robot_.get_position_name(position_index);

      msg.joint_position[i] = static_cast<float>(q[position_index]);
      msg.joint_velocity[i] = static_cast<float>(v[velocity_index]);
      msg.joint_effort[i] = 0;
      i++;
    }
  }

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<T>& descriptor) const override {
    return systems::AbstractValue::Make<bot_core::robot_state_t>(
        bot_core::robot_state_t());
  }

  inline const systems::InputPortDescriptor<T>& get_input_port_state() const {
    return this->get_input_port(input_port_index_state_);
  }

  inline const systems::OutputPortDescriptor<T>& get_output_port_msg() const {
    return this->get_output_port(output_port_index_msg_);
  }

 private:
  const RigidBodyTree<T>& robot_;
  const RigidBody<T>& base_body_;
  int input_port_index_state_{0};
  int output_port_index_msg_{0};
};


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

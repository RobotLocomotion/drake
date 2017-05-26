#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/lcm/translator_base.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class StateVectorAndHumanoidStatusTranslator final
    : public drake::lcm::TranslatorBase<
          systems::BasicVector<double>, HumanoidStatus> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateVectorAndHumanoidStatusTranslator)

  StateVectorAndHumanoidStatusTranslator(const RigidBodyTree<double>& robot,
                                         const std::string& alias_group_path)
      : default_status_(robot, param_parsers::RigidBodyTreeAliasGroups<double>(
                                   robot, alias_group_path)),
        default_state_(VectorX<double>::Zero(robot.get_num_positions() +
                                             robot.get_num_velocities())) {}

  const systems::BasicVector<double>& get_default_data() const override {
    return default_state_;
  }

  const HumanoidStatus& get_default_msg() const override {
    return default_status_;
  }

  void Encode(double time, const systems::BasicVector<double>& data,
              HumanoidStatus* msg) const override;

  void Decode(const HumanoidStatus& msg, double* time,
              systems::BasicVector<double>* data) const override;

 private:
  const HumanoidStatus default_status_;
  const systems::BasicVector<double> default_state_;
};

class HumanoidStatusAndRobotStateMsgTranslator final
    : public drake::lcm::TranslatorBase<HumanoidStatus,
                                        bot_core::robot_state_t> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HumanoidStatusAndRobotStateMsgTranslator)

  HumanoidStatusAndRobotStateMsgTranslator(const RigidBodyTree<double>& robot,
                                           const std::string& alias_group_path)
      : translator_(robot),
        default_status_(robot, param_parsers::RigidBodyTreeAliasGroups<double>(
                                   robot, alias_group_path)) {
    tmp_position_.resize(robot.get_num_positions());
    tmp_velocity_.resize(robot.get_num_velocities());
    tmp_joint_torque_.resize(robot.get_num_actuators());

    translator_.InitializeMessage(&default_msg_);
  }

  const HumanoidStatus& get_default_data() const override {
    return default_status_;
  }

  const bot_core::robot_state_t& get_default_msg() const override {
    return default_msg_;
  }

  void Encode(double time, const HumanoidStatus& data,
              bot_core::robot_state_t* msg) const override;

  void Decode(const bot_core::robot_state_t& msg, double* time,
              HumanoidStatus* data) const override;

 private:
  const manipulation::RobotStateLcmMessageTranslator translator_;
  const HumanoidStatus default_status_;
  bot_core::robot_state_t default_msg_;

  // Only used as temporary buffers for decoding.
  mutable VectorX<double> tmp_position_;
  mutable VectorX<double> tmp_velocity_;
  mutable VectorX<double> tmp_joint_torque_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

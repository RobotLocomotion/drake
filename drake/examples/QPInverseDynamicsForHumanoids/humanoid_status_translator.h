#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/lcm/translator_base.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A Translator between raw state vector (systems::BasicVector<double>) and
 * HumanoidStatus. Note that since the raw state does not contain any time
 * information, the template arguments for drake::lcm::TranslatorBase are
 * chosen as DataType = systems::BasicVector<double>, and
 * MsgType = HumanoidStatus. This way, when combined with a
 * systems::lcm::LcmEncoderSystem for translation, context's time can be
 * encoded properly. See documentation for drake::lcm::TranslatorBase for
 * more details.
 */
class StateVectorAndHumanoidStatusTranslator final
    : public drake::lcm::TranslatorBase<systems::BasicVector<double>,
                                        HumanoidStatus> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateVectorAndHumanoidStatusTranslator)

  /**
   * Constructor. @p robot's life span must be longer than this.
   */
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

  /**
   * @p time is used to update the timing in @p msg, and @p data is used to
   * update the kinematics of @p msg. The rest in @p msg is not changed.
   * Assumes that @p data has the same dimension as @p msg's rigid body tree's
   * state. @p msg cannot be nullptr.
   */
  void Encode(const systems::BasicVector<double>& data,
              HumanoidStatus* msg) const override;

  /**
   * @p time will be set to the time in @p msg. The state (q, v) are copied
   * into @p data. Assumes that @p data has the same dimension as @p msg's
   * rigid body tree's state. @p time and @p data cannot be nullptr.
   */
  void Decode(const HumanoidStatus& msg,
              systems::BasicVector<double>* data) const override;

  void EncodeTime(double time, HumanoidStatus* msg) const override {
    msg->UpdateTime(time);
  }

  void DecodeTime(const HumanoidStatus& msg, double* time) const override {
    *time = msg.time();
  }

 private:
  const HumanoidStatus default_status_;
  const systems::BasicVector<double> default_state_;
};

/**
 * A Translator between HumanoidStatus and bot_core::robot_state_t. This is a
 * wrapper class on around manipulation::RobotStateLcmMessageTranslator. Refer
 * to the latter for more details.
 */
class HumanoidStatusAndRobotStateMsgTranslator final
    : public drake::lcm::TranslatorBase<HumanoidStatus,
                                        bot_core::robot_state_t> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HumanoidStatusAndRobotStateMsgTranslator)

  /**
   * Constructor. @p robot's life span must be longer than this.
   */
  HumanoidStatusAndRobotStateMsgTranslator(const RigidBodyTree<double>& robot,
                                           const std::string& alias_group_path)
      : robot_state_translator_(robot),
        default_status_(robot, param_parsers::RigidBodyTreeAliasGroups<double>(
                                   robot, alias_group_path)) {
    tmp_position_.resize(robot.get_num_positions());
    tmp_velocity_.resize(robot.get_num_velocities());
    tmp_joint_torque_.resize(robot.get_num_actuators());

    robot_state_translator_.InitializeMessage(&default_msg_);
  }

  const HumanoidStatus& get_default_data() const override {
    return default_status_;
  }

  const bot_core::robot_state_t& get_default_msg() const override {
    return default_msg_;
  }

  /**
   * @p msg cannot be nullptr.
   */
  void Encode(const HumanoidStatus& data,
              bot_core::robot_state_t* msg) const override;

  /**
   * @p time and @p data cannot be nullptr.
   */
  void Decode(const bot_core::robot_state_t& msg,
              HumanoidStatus* data) const override;

  void EncodeTime(double time, bot_core::robot_state_t* msg) const override {
    msg->utime = static_cast<int>(time * 1e6);
  }

  void DecodeTime(const bot_core::robot_state_t& msg,
                  double* time) const override {
    *time = static_cast<double>(msg.utime) / 1e6;
  }

 private:
  const manipulation::RobotStateLcmMessageTranslator robot_state_translator_;
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

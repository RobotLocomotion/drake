#pragma once

#include <memory>
#include <string>

#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

// TODO(siyuan): move this to /manipulation/util

/**
 * A System block that takes state vector and output a bot_core::robot_state_t
 * message. For the encoded `message`, `message.pose` will always be
 * X_WB (pose of `B` in `W`), where `W` is the world frame, and `B` is the
 * first non-world rigid body's body frame in the given rigid body tree.
 * Similarly, `message.twist` = V_WB is the spatial velocity of `B` relative
 * to `W` expressed in `W`. If the first non-world rigid body is rigidly
 * attached to the world, V_WB = 0. The joint section of `message` excludes all
 * degrees of freedom that correspond to the floating base (if one exists),
 * and will be in the order of the rigid body tree's generalized position.
 * Note that the `message.joint_effort` will be set to zero as this class
 * only encodes kinematics information.
 * See manipulation::RobotStateLcmMessageTranslator for more details.
 */
template <typename T>
class OracularStateEstimation : public systems::LeafSystem<T> {
 public:
  /**
   * Constructor for OracularStateEstimation.
   *
   * @param robot, Reference to the RigidBodyTree. The life span of @p robot
   * needs to be longer than this instance. Also note that the generated LCM
   * messages will contain every joint in @p robot.
   */
  explicit OracularStateEstimation(const RigidBodyTree<T>& robot)
      : robot_(robot), translator_(robot_) {
    input_port_index_state_ =
        this->DeclareInputPort(
                systems::kVectorValued,
                robot.get_num_positions() + robot.get_num_velocities())
            .get_index();
    output_port_index_msg_ =
        this->DeclareAbstractOutputPort(
                &OracularStateEstimation::MakeRobotState,
                &OracularStateEstimation::OutputRobotState)
            .get_index();
  }

  void OutputRobotState(const systems::Context<T>& context,
                        bot_core::robot_state_t* output) const {
    const systems::BasicVector<T>* state =
        this->EvalVectorInput(context, input_port_index_state_);

    VectorX<T> q = state->get_value().head(robot_.get_num_positions());
    VectorX<T> v = state->get_value().tail(robot_.get_num_velocities());

    bot_core::robot_state_t& msg = *output;
    msg.utime = static_cast<int64_t>(context.get_time() * 1e6);
    translator_.EncodeMessageKinematics(
        state->get_value().head(robot_.get_num_positions()),
        state->get_value().tail(robot_.get_num_velocities()),
        &msg);
  }

  bot_core::robot_state_t MakeRobotState() const {
    bot_core::robot_state_t msg;
    // Resize and zeros the message.
    translator_.InitializeMessage(&msg);
    return msg;
  }

  inline const systems::InputPortDescriptor<T>& get_input_port_state() const {
    return this->get_input_port(input_port_index_state_);
  }

  inline const systems::OutputPort<T>& get_output_port_msg() const {
    return this->get_output_port(output_port_index_msg_);
  }

 private:
  const RigidBodyTree<T>& robot_;
  const manipulation::RobotStateLcmMessageTranslator translator_;

  int input_port_index_state_{0};
  int output_port_index_msg_{0};
};


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

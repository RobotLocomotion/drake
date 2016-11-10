#include "drake/examples/Valkyrie/robot_state_encoder.h"
#include "drake/common/constants.h"
#include "drake/examples/Valkyrie/robot_state_lcmtype_util.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/util/drakeUtil.h"
#include "drake/util/lcmUtil.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

using bot_core::robot_state_t;

using Eigen::Dynamic;
using Eigen::Index;
using Eigen::Matrix;
using Eigen::Isometry3d;
using Eigen::Translation3d;

namespace drake {
namespace systems {

RobotStateEncoder::RobotStateEncoder(const RigidBodyTree<double>& tree)
    : tree_(CheckTreeIsRobotStateLcmTypeCompatible(tree)),
      floating_body_(tree.bodies[1]->getJoint().is_floating()
                         ? tree.bodies[1].get()
                         : nullptr),
      lcm_message_port_index_(
          DeclareAbstractOutputPort(kContinuousSampling).get_index()),
      kinematics_results_port_index_(
          DeclareAbstractInputPort(kContinuousSampling).get_index()),
      effort_port_indices_(DeclareEffortInputPorts()),
      foot_wrench_port_indices_(DeclareWrenchInputPorts()),
      hand_wrench_port_indices_(DeclareWrenchInputPorts()) {
  set_name("RobotStateEncoder");
}

RobotStateEncoder::~RobotStateEncoder() {}

void RobotStateEncoder::EvalOutput(const Context<double>& context,
                                   SystemOutput<double>* output) const {
  auto& message = output->GetMutableData(lcm_message_port_index_)
                      ->GetMutableValue<robot_state_t>();
  message.utime = static_cast<int64_t>(context.get_time() * 1e6);
  SetStateAndEfforts(context, &message);
  SetForceTorque(context, &message);
}

std::unique_ptr<SystemOutput<double>> RobotStateEncoder::AllocateOutput(
    const Context<double>& context) const {
  auto output = make_unique<LeafSystemOutput<double>>();

  auto data = make_unique<Value<robot_state_t>>(robot_state_t());
  output->add_port(move(data));

  return std::unique_ptr<SystemOutput<double>>(output.release());
}

const SystemPortDescriptor<double>& RobotStateEncoder::lcm_message_port()
    const {
  return get_output_port(lcm_message_port_index_);
}

const SystemPortDescriptor<double>& RobotStateEncoder::kinematics_results_port()
    const {
  return get_input_port(kinematics_results_port_index_);
}

const SystemPortDescriptor<double>& RobotStateEncoder::effort_port(
    const RigidBodyActuator& actuator) const {
  return get_input_port(effort_port_indices_.at(&actuator));
}

const SystemPortDescriptor<double>& RobotStateEncoder::foot_contact_wrench_port(
    const Side& side) const {
  return get_input_port(foot_wrench_port_indices_.at(side));
}

const SystemPortDescriptor<double>& RobotStateEncoder::hand_contact_wrench_port(
    const Side& side) const {
  return get_input_port(hand_wrench_port_indices_.at(side));
}

std::map<const RigidBodyActuator*, int>
RobotStateEncoder::DeclareEffortInputPorts() {
  std::map<const RigidBodyActuator*, int> ret;

  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int actuator_effort_length = 1;
  for (const auto& actuator : tree_.actuators) {
    ret[&actuator] = DeclareInputPort(kVectorValued, actuator_effort_length,
                                      kContinuousSampling)
                         .get_index();
  }
  return ret;
}

std::map<Side, int> RobotStateEncoder::DeclareWrenchInputPorts() {
  std::map<Side, int> ret;
  for (const auto& side : Side::values) {
    ret[side] = DeclareInputPort(kVectorValued, kTwistSize, kContinuousSampling)
                    .get_index();
  }
  return ret;
}

void RobotStateEncoder::SetStateAndEfforts(const Context<double>& context,
                                           robot_state_t* message) const {
  const auto& kinematics_results =
      EvalAbstractInput(context, kinematics_results_port_index_)
          ->GetValue<KinematicsResults<double>>();

  if (floating_body_) {
    // Pose of floating body with respect to world.
    Isometry3d floating_body_to_world =
        kinematics_results.get_pose_in_world(*floating_body_);
    EncodePose(floating_body_to_world, message->pose);

    // Twist of floating body with respect to world.
    // To match usage of robot_state_t throughout OpenHumanoids code, use
    // twist frame that has the same orientation as world
    // frame, but the same origin as the floating body frame.
    TwistVector<double> floating_body_twist =
        kinematics_results.get_twist_in_world_aligned_body_frame(
            *floating_body_);
    EncodeTwist(floating_body_twist, message->twist);
  } else {
    EncodePose(Isometry3d::Identity(), message->pose);
    EncodeTwist(Vector6<double>::Zero(), message->twist);
  }

  // Joint names, positions, velocities, and efforts.
  // Note: the order of the actuators in the rigid body tree determines the
  // order of the joint_name, joint_position, joint_velocity, and
  // joint_effort fields.
  message->joint_name.clear();
  message->joint_position.clear();
  message->joint_velocity.clear();
  message->joint_effort.clear();
  for (const auto& actuator : tree_.actuators) {
    const auto& body = *actuator.body_;
    int effort_port_index = effort_port_indices_.at(&actuator);

    // To match usage of robot_state_t throughout OpenHumanoids code, set
    // joint_names field to position coordinate names.
    int position_index = body.get_position_start_index();
    message->joint_name.push_back(tree_.get_position_name(position_index));

    auto position =
        static_cast<float>(kinematics_results.get_joint_position(body)[0]);
    auto velocity =
        static_cast<float>(kinematics_results.get_joint_velocity(body)[0]);
    auto effort = static_cast<float>(
        EvalVectorInput(context, effort_port_index)->GetAtIndex(0));

    message->joint_position.push_back(position);
    message->joint_velocity.push_back(velocity);
    message->joint_effort.push_back(effort);
  }
  message->num_joints = static_cast<int16_t>(message->joint_name.size());
  message->utime = static_cast<int64_t>(1e6 * context.get_time());
}

void RobotStateEncoder::SetForceTorque(const Context<double>& context,
                                       bot_core::robot_state_t* message) const {
  auto& force_torque = message->force_torque;
  {
    auto left_foot_wrench =
        EvalVectorInput(context, foot_wrench_port_indices_.at(Side::LEFT))
            ->get_value();
    force_torque.l_foot_force_z =
        static_cast<float>(left_foot_wrench[kForceZIndex]);
    force_torque.l_foot_torque_x =
        static_cast<float>(left_foot_wrench[kTorqueXIndex]);
    force_torque.l_foot_torque_y =
        static_cast<float>(left_foot_wrench[kTorqueYIndex]);
  }
  {
    auto right_foot_wrench =
        EvalVectorInput(context, foot_wrench_port_indices_.at(Side::RIGHT))
            ->get_value();
    force_torque.r_foot_force_z =
        static_cast<float>(right_foot_wrench[kForceZIndex]);
    force_torque.r_foot_torque_x =
        static_cast<float>(right_foot_wrench[kTorqueXIndex]);
    force_torque.r_foot_torque_y =
        static_cast<float>(right_foot_wrench[kTorqueYIndex]);
  }
  {
    auto left_hand_wrench =
        EvalVectorInput(context, hand_wrench_port_indices_.at(Side::LEFT))
            ->get_value();
    eigenVectorToCArray(left_hand_wrench.head<kSpaceDimension>(),
                        force_torque.l_hand_torque);
    eigenVectorToCArray(left_hand_wrench.tail<kSpaceDimension>(),
                        force_torque.l_hand_force);
  }
  {
    auto right_hand_wrench =
        EvalVectorInput(context, hand_wrench_port_indices_.at(Side::RIGHT))
            ->get_value();
    eigenVectorToCArray(right_hand_wrench.head<kSpaceDimension>(),
                        force_torque.r_hand_torque);
    eigenVectorToCArray(right_hand_wrench.tail<kSpaceDimension>(),
                        force_torque.r_hand_force);
  }
}

}  // namespace systems
}  // namespace drake

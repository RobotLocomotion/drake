#include <list>

#include "drake/examples/Valkyrie/robot_state_encoder.h"
#include "drake/common/constants.h"
#include "drake/examples/Valkyrie/robot_state_lcmtype_util.h"
#include "drake/multibody/rigid_body_plant/contact_force.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
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

RobotStateEncoder::RobotStateEncoder(
    const RigidBodyTree<double>& tree,
    const std::vector<std::string>& ft_sensor_attached_body_names,
    const std::vector<Isometry3<double>>& ft_sensor_offsets_in_body_frame)
    : tree_(CheckTreeIsRobotStateLcmTypeCompatible(tree)),
      floating_body_(tree.bodies[1]->getJoint().is_floating()
                         ? tree.bodies[1].get()
                         : nullptr),
      lcm_message_port_index_(
          DeclareAbstractOutputPort(kContinuousSampling).get_index()),
      kinematics_results_port_index_(
          DeclareAbstractInputPort(kContinuousSampling).get_index()),
      contact_results_port_index_(
          DeclareAbstractInputPort(kContinuousSampling).get_index()),
      effort_port_indices_(DeclareEffortInputPorts()),
      world_(*tree_.FindBody("world")) {
  if (ft_sensor_attached_body_names.size() !=
      ft_sensor_offsets_in_body_frame.size()) {
    throw std::runtime_error(
        "FT sensor size doesn't match FT sensor offset size");
  }

  force_torque_sensors_.resize(ft_sensor_offsets_in_body_frame.size());
  for (size_t i = 0; i < ft_sensor_offsets_in_body_frame.size(); ++i) {
    const std::string& name = ft_sensor_attached_body_names[i];
    force_torque_sensors_[i] =
        std::pair<const RigidBody<double>*, Isometry3<double>>(
            tree_.FindBody(name), ft_sensor_offsets_in_body_frame[i]);
    // TODO(siyuan.feng): this needs to not be hard coded.
    if (name.compare("leftFoot") == 0)
      l_foot_ft_sensor_idx_ = static_cast<int>(i);
    if (name.compare("rightFoot") == 0)
      r_foot_ft_sensor_idx_ = static_cast<int>(i);
    if (name.compare("leftPalm") == 0)
      l_hand_ft_sensor_idx_ = static_cast<int>(i);
    if (name.compare("rightPalm") == 0)
      r_hand_ft_sensor_idx_ = static_cast<int>(i);
  }

  set_name("RobotStateEncoder");
}

RobotStateEncoder::~RobotStateEncoder() {}

void RobotStateEncoder::EvalOutput(const Context<double>& context,
                                   SystemOutput<double>* output) const {
  auto& message = output->GetMutableData(lcm_message_port_index_)
                      ->GetMutableValue<robot_state_t>();
  message.utime = static_cast<int64_t>(context.get_time() * 1e6);

  const auto& contact_results =
      EvalAbstractInput(context, contact_results_port_index_)
          ->GetValue<ContactResults<double>>();

  const auto& kinematics_results =
      EvalAbstractInput(context, kinematics_results_port_index_)
          ->GetValue<KinematicsResults<double>>();

  SetStateAndEfforts(kinematics_results, context, &message);
  SetForceTorque(kinematics_results, contact_results, &message);
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

const SystemPortDescriptor<double>& RobotStateEncoder::contact_results_port()
    const {
  return get_input_port(contact_results_port_index_);
}

const SystemPortDescriptor<double>& RobotStateEncoder::effort_port(
    const RigidBodyActuator& actuator) const {
  return get_input_port(effort_port_indices_.at(&actuator));
}

std::map<const RigidBodyActuator*, int>
RobotStateEncoder::DeclareEffortInputPorts() {
  std::map<const RigidBodyActuator*, int> ret;

  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int actuator_effort_length = 1;
  for (const auto& actuator : tree_.actuators) {
    ret[&actuator] = DeclareInputPort(kVectorValued, actuator_effort_length,
                                      kContinuousSampling).get_index();
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

void RobotStateEncoder::SetStateAndEfforts(
    const KinematicsResults<double>& kinematics_results,
    const Context<double>& context, robot_state_t* message) const {
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

SpatialForce<double>
RobotStateEncoder::GetSpatialForceActingOnBody1ByBody2InBody1Frame(
    const KinematicsResults<double>& kinematics_results,
    const ContactResults<double>& contact_results,
    const RigidBody<double>& body1, const RigidBody<double>& body2) const {
  std::list<ContactForce<double>> contact_forces;
  for (int i = 0; i < contact_results.get_num_contacts(); ++i) {
    const ContactInfo<double>& contact_info =
        contact_results.get_contact_info(i);
    const RigidBody<double>& b1 =
        *tree_.FindBody(contact_info.get_element_id_1());
    const RigidBody<double>& b2 =
        *tree_.FindBody(contact_info.get_element_id_2());
    if (b1.get_name().compare(body1.get_name()) == 0 &&
        b2.get_name().compare(body2.get_name()) == 0) {
      contact_forces.push_back(contact_info.get_resultant_force());
    }
    if (b2.get_name().compare(body1.get_name()) == 0 &&
        b1.get_name().compare(body2.get_name()) == 0) {
      contact_forces.push_back(
          contact_info.get_resultant_force().reaction_force());
    }
  }

  ContactResultantForceCalculator<double> calc;
  const Vector3<double>& reference_point =
      kinematics_results.get_pose_in_world(body1).translation();

  for (const ContactForce<double>& f : contact_forces) calc.AddForce(f);

  SpatialForce<double> wrench_in_world_aligned_body_frame =
      calc.ComputeResultant(reference_point).get_spatial_force();
  Isometry3<double> world_aligned_to_body_frame(Isometry3<double>::Identity());
  world_aligned_to_body_frame.linear() =
      kinematics_results.get_pose_in_world(body1).linear().transpose();
  return transformSpatialForce(world_aligned_to_body_frame,
                               wrench_in_world_aligned_body_frame);
}

void RobotStateEncoder::SetForceTorque(
    const KinematicsResults<double>& kinematics_results,
    const ContactResults<double>& contact_results,
    bot_core::robot_state_t* message) const {
  std::vector<SpatialForce<double>> wrench_in_sensor_frame(
      force_torque_sensors_.size());

  for (size_t i = 0; i < force_torque_sensors_.size(); ++i) {
    const RigidBody<double>& body = *force_torque_sensors_[i].first;
    SpatialForce<double> wrench =
        GetSpatialForceActingOnBody1ByBody2InBody1Frame(
            kinematics_results, contact_results, body, world_);
    wrench_in_sensor_frame[i] =
        transformSpatialForce(force_torque_sensors_[i].second, wrench);
  }

  auto& force_torque = message->force_torque;

  if (l_foot_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& wrench =
        wrench_in_sensor_frame.at(l_foot_ft_sensor_idx_);
    force_torque.l_foot_force_z = static_cast<float>(wrench[kForceZIndex]);
    force_torque.l_foot_torque_x = static_cast<float>(wrench[kTorqueXIndex]);
    force_torque.l_foot_torque_y = static_cast<float>(wrench[kTorqueYIndex]);
  }
  if (r_foot_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& wrench =
        wrench_in_sensor_frame.at(r_foot_ft_sensor_idx_);
    force_torque.r_foot_force_z = static_cast<float>(wrench[kForceZIndex]);
    force_torque.r_foot_torque_x = static_cast<float>(wrench[kTorqueXIndex]);
    force_torque.r_foot_torque_y = static_cast<float>(wrench[kTorqueYIndex]);
  }
  if (l_hand_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& wrench =
        wrench_in_sensor_frame.at(l_hand_ft_sensor_idx_);
    eigenVectorToCArray(wrench.head<kSpaceDimension>(),
                        force_torque.l_hand_torque);
    eigenVectorToCArray(wrench.tail<kSpaceDimension>(),
                        force_torque.l_hand_force);
  }
  if (r_hand_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& wrench =
        wrench_in_sensor_frame.at(r_hand_ft_sensor_idx_);
    eigenVectorToCArray(wrench.head<kSpaceDimension>(),
                        force_torque.r_hand_torque);
    eigenVectorToCArray(wrench.tail<kSpaceDimension>(),
                        force_torque.r_hand_force);
  }
}

}  // namespace systems
}  // namespace drake

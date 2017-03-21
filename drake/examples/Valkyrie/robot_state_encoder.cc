#include "drake/examples/Valkyrie/robot_state_encoder.h"

#include <list>

#include "drake/common/constants.h"
#include "drake/examples/Valkyrie/robot_state_lcmtype_util.h"
#include "drake/multibody/rigid_body_plant/contact_force.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/util/drakeGeometryUtil.h"
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
    const std::vector<RigidBodyFrame<double>>& ft_sensor_info)
    : tree_(CheckTreeIsRobotStateLcmTypeCompatible(tree)),
      floating_body_(tree.bodies[1]->getJoint().is_floating()
                         ? tree.bodies[1].get()
                         : nullptr),
      lcm_message_port_index_(DeclareAbstractOutputPort().get_index()),
      kinematics_results_port_index_(DeclareAbstractInputPort().get_index()),
      contact_results_port_index_(DeclareAbstractInputPort().get_index()),
      effort_port_indices_(DeclareEffortInputPorts()),
      force_torque_sensor_info_(ft_sensor_info) {
  int sensor_idx = 0;
  for (const auto& sensor : force_torque_sensor_info_) {
    const std::string& name = sensor.get_rigid_body().get_name();

    // TODO(siyuan.feng): this needs to not be hard coded.
    if (name.compare("leftFoot") == 0) l_foot_ft_sensor_idx_ = sensor_idx;
    if (name.compare("rightFoot") == 0) r_foot_ft_sensor_idx_ = sensor_idx;
    if (name.compare("leftPalm") == 0) l_hand_ft_sensor_idx_ = sensor_idx;
    if (name.compare("rightPalm") == 0) r_hand_ft_sensor_idx_ = sensor_idx;

    sensor_idx++;
  }

  set_name("RobotStateEncoder");
}

RobotStateEncoder::~RobotStateEncoder() {}

void RobotStateEncoder::DoCalcOutput(const Context<double>& context,
                                     SystemOutput<double>* output) const {
  auto& message = output->GetMutableData(lcm_message_port_index_)
                      ->GetMutableValue<robot_state_t>();
  message.utime = static_cast<int64_t>(context.get_time() * 1e6);

  // TODO(siyuan.feng): I explicitly evaluated kinematics and contacts
  // separately here to avoid excessive calls given the same context.
  // This shouldn't be necessary when cache is correctly implemented.
  const auto& contact_results =
      EvalAbstractInput(context, contact_results_port_index_)
          ->GetValue<ContactResults<double>>();

  const auto& kinematics_results =
      EvalAbstractInput(context, kinematics_results_port_index_)
          ->GetValue<KinematicsResults<double>>();

  SetStateAndEfforts(kinematics_results, context, &message);
  SetForceTorque(kinematics_results, contact_results, &message);
}

std::unique_ptr<AbstractValue> RobotStateEncoder::AllocateOutputAbstract(
    const OutputPortDescriptor<double>& descriptor) const {
  return make_unique<Value<robot_state_t>>(robot_state_t());
}

const OutputPortDescriptor<double>& RobotStateEncoder::lcm_message_port()
    const {
  return get_output_port(lcm_message_port_index_);
}

const InputPortDescriptor<double>& RobotStateEncoder::kinematics_results_port()
    const {
  return get_input_port(kinematics_results_port_index_);
}

const InputPortDescriptor<double>& RobotStateEncoder::contact_results_port()
    const {
  return get_input_port(contact_results_port_index_);
}

const InputPortDescriptor<double>& RobotStateEncoder::effort_port(
    const RigidBodyActuator& actuator) const {
  return get_input_port(effort_port_indices_.at(&actuator));
}

std::map<const RigidBodyActuator*, int>
RobotStateEncoder::DeclareEffortInputPorts() {
  std::map<const RigidBodyActuator*, int> ret;

  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int actuator_effort_length = 1;
  for (const auto& actuator : tree_.actuators) {
    ret[&actuator] =
        DeclareInputPort(kVectorValued, actuator_effort_length).get_index();
  }
  return ret;
}

std::map<Side, int> RobotStateEncoder::DeclareWrenchInputPorts() {
  std::map<Side, int> ret;
  for (const auto& side : Side::values) {
    ret[side] = DeclareInputPort(kVectorValued, kTwistSize).get_index();
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
    const RigidBody<double>* b1 =
        tree_.FindBody(contact_info.get_element_id_1());
    const RigidBody<double>* b2 =
        tree_.FindBody(contact_info.get_element_id_2());
    // These are point forces in the world frame, and are applied at points
    // in the world frame.
    // TODO(siyuan.feng): replace pointer comparison with a proper one that
    // compares rigid bodies.
    if (b1 == &body1 && b2 == &body2) {
      contact_forces.push_back(contact_info.get_resultant_force());
    } else if (b2 == &body1 && b1 == &body2) {
      contact_forces.push_back(
          contact_info.get_resultant_force().get_reaction_force());
    }
  }

  ContactResultantForceCalculator<double> calc;
  const Vector3<double>& reference_point =
      kinematics_results.get_pose_in_world(body1).translation();

  for (const ContactForce<double>& f : contact_forces) {
    calc.AddForce(f);
  }

  SpatialForce<double> spatial_force_in_world_aligned_body_frame =
      calc.ComputeResultant(reference_point).get_spatial_force();
  Isometry3<double> world_aligned_to_body_frame(Isometry3<double>::Identity());
  world_aligned_to_body_frame.linear() =
      kinematics_results.get_pose_in_world(body1).linear().transpose();
  return transformSpatialForce(world_aligned_to_body_frame,
                               spatial_force_in_world_aligned_body_frame);
}

void RobotStateEncoder::SetForceTorque(
    const KinematicsResults<double>& kinematics_results,
    const ContactResults<double>& contact_results,
    bot_core::robot_state_t* message) const {
  std::vector<SpatialForce<double>> spatial_force_in_sensor_frame(
      force_torque_sensor_info_.size());

  for (size_t i = 0; i < force_torque_sensor_info_.size(); ++i) {
    const RigidBody<double>& body =
        force_torque_sensor_info_[i].get_rigid_body();
    SpatialForce<double> spatial_force =
        GetSpatialForceActingOnBody1ByBody2InBody1Frame(
            kinematics_results, contact_results, body, tree_.world());
    Isometry3<double> body_to_sensor =
        force_torque_sensor_info_[i].get_transform_to_body().inverse();
    spatial_force_in_sensor_frame[i] =
        transformSpatialForce(body_to_sensor, spatial_force);
  }

  auto& force_torque = message->force_torque;

  if (l_foot_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& spatial_force =
        spatial_force_in_sensor_frame.at(l_foot_ft_sensor_idx_);
    force_torque.l_foot_force_z =
        static_cast<float>(spatial_force[kForceZIndex]);
    force_torque.l_foot_torque_x =
        static_cast<float>(spatial_force[kTorqueXIndex]);
    force_torque.l_foot_torque_y =
        static_cast<float>(spatial_force[kTorqueYIndex]);
  }
  if (r_foot_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& spatial_force =
        spatial_force_in_sensor_frame.at(r_foot_ft_sensor_idx_);
    force_torque.r_foot_force_z =
        static_cast<float>(spatial_force[kForceZIndex]);
    force_torque.r_foot_torque_x =
        static_cast<float>(spatial_force[kTorqueXIndex]);
    force_torque.r_foot_torque_y =
        static_cast<float>(spatial_force[kTorqueYIndex]);
  }
  if (l_hand_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& spatial_force =
        spatial_force_in_sensor_frame.at(l_hand_ft_sensor_idx_);
    eigenVectorToCArray(spatial_force.head<kSpaceDimension>(),
                        force_torque.l_hand_torque);
    eigenVectorToCArray(spatial_force.tail<kSpaceDimension>(),
                        force_torque.l_hand_force);
  }
  if (r_hand_ft_sensor_idx_ != -1) {
    const SpatialForce<double>& spatial_force =
        spatial_force_in_sensor_frame.at(r_hand_ft_sensor_idx_);
    eigenVectorToCArray(spatial_force.head<kSpaceDimension>(),
                        force_torque.r_hand_torque);
    eigenVectorToCArray(spatial_force.tail<kSpaceDimension>(),
                        force_torque.r_hand_force);
  }
}

}  // namespace systems
}  // namespace drake

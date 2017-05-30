#include "drake/examples/Valkyrie/robot_state_encoder.h"

#include <list>

#include "drake/common/constants.h"
#include "drake/multibody/rigid_body_plant/contact_force.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
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
    : translator_(tree),
      lcm_message_port_index_(
          DeclareAbstractOutputPort(&RobotStateEncoder::MakeRobotState,
                                    &RobotStateEncoder::OutputRobotState)
              .get_index()),
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

void RobotStateEncoder::OutputRobotState(const Context<double>& context,
                                         robot_state_t* output) const {
  auto& message = *output;
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

robot_state_t RobotStateEncoder::MakeRobotState() const {
  robot_state_t msg;
  translator_.InitializeMessage(&msg);
  return msg;
}

const OutputPort<double>& RobotStateEncoder::lcm_message_port()
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
  for (const auto& actuator : translator_.get_robot().actuators) {
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
  message->utime = static_cast<int64_t>(1e6 * context.get_time());
  translator_.EncodeMessageKinematics(
      kinematics_results.get_cache().getQ(),
      kinematics_results.get_cache().getV(),
      message);

  VectorX<double> torque(translator_.get_robot().get_num_actuators());
  int actuator_index = 0;
  for (const auto& actuator : translator_.get_robot().actuators) {
    int effort_port_index = effort_port_indices_.at(&actuator);
    torque(actuator_index++) =
        EvalVectorInput(context, effort_port_index)->GetAtIndex(0);
  }

  translator_.EncodeMessageTorque(torque, message);
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
        translator_.get_robot().FindBody(contact_info.get_element_id_1());
    const RigidBody<double>* b2 =
        translator_.get_robot().FindBody(contact_info.get_element_id_2());
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
            kinematics_results, contact_results, body,
            translator_.get_robot().world());
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

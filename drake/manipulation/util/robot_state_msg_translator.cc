#include "drake/manipulation/util/robot_state_msg_translator.h"

#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {

namespace {

// For every ith name in @p names, if it is present in @p name_to_index, set
// msg_values[i] = values[name_to_index.find(names[i])]
void EncodeValue(const std::unordered_map<std::string, int>& name_to_index,
                 const std::vector<std::string>& names,
                 const Eigen::Ref<const VectorX<double>>& values,
                 std::vector<float>* msg_values) {
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string& name = names[i];
    // Ignores joints we don't know about.
    auto it = name_to_index.find(name);
    if (it != name_to_index.end()) {
      DRAKE_ASSERT(it->second < values.size() && it->second >= 0);
      msg_values->at(i) = values(it->second);
    }
  }
}

// For every ith name in @p names, if it is present in @p name_to_index, set
// values[name_to_index.find(names[i])] = msg_values[i]
void DecodeJointValue(const std::unordered_map<std::string, int>& name_to_index,
                      const std::vector<std::string>& names,
                      const std::vector<float>& msg_values,
                      Eigen::Ref<VectorX<double>> values) {
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string& name = names[i];
    // Ignores joints we don't know about.
    auto it = name_to_index.find(name);
    if (it != name_to_index.end()) {
      DRAKE_ASSERT(it->second < values.size() && it->second >= 0);
      values(it->second) = msg_values.at(i);
    }
  }
}

}  // namespace

RobotStateLcmMessageTranslator::RobotStateLcmMessageTranslator(
    const RigidBodyTree<double>& robot)
    : robot_(CheckTreeIsRobotStateLcmTypeCompatible(robot)),
      floating_base_(robot_.bodies[1]->getJoint().is_floating()
                         ? robot_.bodies[1].get()
                         : nullptr) {
  std::unordered_map<const RigidBody<double>*, std::string> body_to_joint_name;

  for (const auto& body : robot.bodies) {
    if (body->has_parent_body()) {
      const auto& joint = body->getJoint();
      if (!joint.is_fixed() && !joint.is_floating()) {
        // Assuming for the non floating joints, the sizes for q and v are
        // both 1.
        int position_index = body->get_position_start_index();
        int velocity_index = body->get_velocity_start_index();
        DRAKE_ASSERT(joint.get_num_positions() == 1);
        DRAKE_ASSERT(joint.get_num_velocities() == 1);

        // The convention is to use position coordinate name as joint name.
        const std::string& name = robot.get_position_name(position_index);

        joint_name_to_q_index_[name] = position_index;
        joint_name_to_v_index_[name] = velocity_index;
        body_to_joint_name[body.get()] = name;
      }
    }
  }

  for (int actuator_index = 0; actuator_index < robot_.get_num_actuators();
       ++actuator_index) {
    const RigidBody<double>* body = robot_.actuators[actuator_index].body_;
    const std::string& joint_name = body_to_joint_name.at(body);
    joint_name_to_actuator_index_[joint_name] = actuator_index;
  }
}

void RobotStateLcmMessageTranslator::DecodeMessageKinematics(
    const bot_core::robot_state_t& msg,
    Eigen::Ref<VectorX<double>> q,
    Eigen::Ref<VectorX<double>> v) const {
  DRAKE_DEMAND(CheckMessageVectorSize(msg));

  DRAKE_DEMAND(q.size() == robot_.get_num_positions());
  DRAKE_DEMAND(v.size() == robot_.get_num_velocities());

  // Floating joint.
  if (floating_base_) {
    // Pose of floating base in the world frame.
    auto X_WB = DecodePose(msg.pose);
    // Spatial velocity of base in the world frame.
    auto V_WB = DecodeTwist(msg.twist);
    const DrakeJoint& floating_joint = floating_base_->getJoint();
    int position_start = floating_base_->get_position_start_index();
    int velocity_start = floating_base_->get_velocity_start_index();

    if (floating_joint.get_num_positions() == 6) {
      // RPY-parameterized floating joint.
      // Translation.
      q.segment<3>(position_start) = X_WB.translation();

      // Orientation.
      auto rpy = math::rotmat2rpy(X_WB.linear());
      q.segment<3>(position_start + 3) = rpy;

      // Translational velocity.
      auto translationdot = V_WB.tail<3>();
      v.segment<3>(velocity_start) = translationdot;

      // Rotational velocity.
      Eigen::Matrix<double, 3, 3> phi;
      typename math::Gradient<decltype(phi), Eigen::Dynamic>::type* dphi =
          nullptr;
      typename math::Gradient<decltype(phi), Eigen::Dynamic, 2>::type* ddphi =
          nullptr;
      angularvel2rpydotMatrix(rpy, phi, dphi, ddphi);
      auto angular_velocity_world = V_WB.head<3>();
      auto rpydot = (phi * angular_velocity_world).eval();
      v.segment<3>(velocity_start + 3) = rpydot;
    } else if (floating_joint.get_num_positions() == 7) {
      // Quaternion-parameterized floating joint.
      // Translation.
      q.segment<3>(position_start) = X_WB.translation();

      // Orientation.
      auto quat = math::rotmat2quat(X_WB.linear());
      q.segment<4>(position_start + 3) = quat;

      // Transform V_WB to the floating base's body frame (V_WB_B).
      // Translational velocity.
      v.segment<3>(velocity_start + 3) =
          X_WB.linear().transpose() * V_WB.tail<3>();
      // Rotational velocity.
      v.segment<3>(velocity_start) =
          X_WB.linear().transpose() * V_WB.head<3>();
    } else {
      DRAKE_ABORT_MSG("Floating joint is neither a RPY or a Quaternion joint.");
    }
  }

  // Non-floating joints.
  DecodeJointValue(joint_name_to_q_index_, msg.joint_name,
                   msg.joint_position, q);
  DecodeJointValue(joint_name_to_v_index_, msg.joint_name,
                   msg.joint_velocity, v);
}

void RobotStateLcmMessageTranslator::InitializeMessage(
    bot_core::robot_state_t* msg) const {
  DRAKE_DEMAND(msg != nullptr);

  // Encode all the other joints assuming they are all 1 dof.
  size_t num_1dof_q = robot_.get_num_positions();
  size_t num_1dof_v = robot_.get_num_velocities();
  if (floating_base_) {
    num_1dof_q -= floating_base_->getJoint().get_num_positions();
    num_1dof_v -= floating_base_->getJoint().get_num_velocities();
  }
  DRAKE_DEMAND(num_1dof_q == num_1dof_v);

  msg->utime = 0;
  EncodePose(Isometry3<double>::Identity(), msg->pose);
  EncodeTwist(Vector6<double>::Zero(), msg->twist);

  // Resizes joints.
  msg->joint_name.resize(num_1dof_q);
  msg->joint_position.resize(num_1dof_q);
  msg->joint_velocity.resize(num_1dof_q);
  msg->joint_effort.resize(num_1dof_q);
  msg->num_joints = static_cast<int16_t>(msg->joint_name.size());

  // Initialize joints.
  int joint_counter = 0;
  for (int i = 0; i < robot_.get_num_positions(); i++) {
    const std::string& name = robot_.get_position_name(i);
    if (joint_name_to_q_index_.find(name) != joint_name_to_q_index_.end()) {
      msg->joint_name[joint_counter] = name;
      msg->joint_position[joint_counter] = 0;
      msg->joint_velocity[joint_counter] = 0;
      msg->joint_effort[joint_counter] = 0;
      joint_counter++;
    }
  }
  DRAKE_DEMAND(joint_counter == msg->num_joints);

  // Sets force torque part to zero.
  msg->force_torque.l_foot_force_z = 0;
  msg->force_torque.l_foot_torque_x = 0;
  msg->force_torque.l_foot_torque_y = 0;
  msg->force_torque.r_foot_force_z = 0;
  msg->force_torque.r_foot_torque_x = 0;
  msg->force_torque.r_foot_torque_y = 0;
  for (int i = 0; i < 3; ++i) {
    msg->force_torque.l_hand_force[i] = 0;
    msg->force_torque.l_hand_torque[i] = 0;
    msg->force_torque.r_hand_force[i] = 0;
    msg->force_torque.r_hand_torque[i] = 0;
  }
}

void RobotStateLcmMessageTranslator::EncodeMessageKinematics(
    const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& v,
    bot_core::robot_state_t* msg) const {
  DRAKE_DEMAND(msg != nullptr);
  DRAKE_DEMAND(q.size() == robot_.get_num_positions());
  DRAKE_DEMAND(v.size() == robot_.get_num_velocities());
  DRAKE_DEMAND(CheckMessageVectorSize(*msg));

  // Encodes the floating base.
  if (floating_base_) {
    Isometry3<double> X_WB;
    Vector6<double> V_WB;

    const DrakeJoint& floating_joint = floating_base_->getJoint();
    const int position_start = floating_base_->get_position_start_index();
    const int velocity_start = floating_base_->get_velocity_start_index();

    if (floating_joint.get_num_positions() == 6) {
      // RPY
      Vector3<double> rpy = q.segment<3>(position_start + 3);
      Vector3<double> rpydot = v.segment<3>(velocity_start + 3);

      X_WB.translation() = q.segment<3>(position_start);
      X_WB.linear() = math::rpy2rotmat(rpy);
      X_WB.makeAffine();

      Matrix3<double> phi = Matrix3<double>::Zero();
      angularvel2rpydotMatrix(rpy, phi, static_cast<Matrix3<double>*>(nullptr),
                              static_cast<Matrix3<double>*>(nullptr));

      auto decomp = Eigen::ColPivHouseholderQR<Matrix3<double>>(phi);
      V_WB.head<3>() = decomp.solve(rpydot);
      V_WB.tail<3>() = v.segment<3>(velocity_start);
    } else if (floating_joint.get_num_positions() == 7) {
      // Quaternion
      X_WB.translation() = q.segment<3>(position_start);
      X_WB.linear() = math::quat2rotmat(q.segment<4>(position_start + 3));
      X_WB.makeAffine();

      // v has velocity in body frame.
      V_WB.head<3>() = X_WB.linear() * v.segment<3>(velocity_start);
      V_WB.tail<3>() = X_WB.linear() * v.segment<3>(velocity_start + 3);
    } else {
      DRAKE_ABORT_MSG("Floating joint is neither a RPY or a Quaternion joint.");
    }

    EncodePose(X_WB, msg->pose);
    EncodeTwist(V_WB, msg->twist);
  } else {
    EncodePose(Isometry3<double>::Identity(), msg->pose);
    EncodeTwist(Vector6<double>::Zero(), msg->twist);
  }

  EncodeValue(joint_name_to_q_index_, msg->joint_name, q,
              &(msg->joint_position));
  EncodeValue(joint_name_to_v_index_, msg->joint_name, v,
              &(msg->joint_velocity));
}

void RobotStateLcmMessageTranslator::EncodeMessageTorque(
    const Eigen::Ref<const VectorX<double>>& torque,
    bot_core::robot_state_t* msg) const {
  DRAKE_DEMAND(msg != nullptr);
  DRAKE_DEMAND(torque.size() == robot_.get_num_actuators());
  DRAKE_DEMAND(CheckMessageVectorSize(*msg));

  EncodeValue(joint_name_to_actuator_index_, msg->joint_name, torque,
              &(msg->joint_effort));
}

void RobotStateLcmMessageTranslator::DecodeMessageTorque(
    const bot_core::robot_state_t& msg,
    Eigen::Ref<VectorX<double>> torque) const {
  DRAKE_DEMAND(CheckMessageVectorSize(msg));

  DRAKE_DEMAND(torque.size() == robot_.get_num_actuators());
  DecodeJointValue(joint_name_to_actuator_index_, msg.joint_name,
                   msg.joint_effort, torque);
}

const RigidBodyTree<double>&
RobotStateLcmMessageTranslator::CheckTreeIsRobotStateLcmTypeCompatible(
    const RigidBodyTree<double>& robot) {
  if (robot.get_num_bodies() < 2) {
    throw std::logic_error("This class assumes at least one non-world body.");
  }

  bool floating_joint_found = false;
  for (const auto& body_ptr : robot.bodies) {
    if (body_ptr->has_parent_body()) {
      const auto& joint = body_ptr->getJoint();
      if (joint.is_floating()) {
        if (floating_joint_found) {
          throw std::logic_error(
              "robot_state_t assumes at most one floating joint.");
        }
        floating_joint_found = true;

        if (body_ptr != robot.bodies[1]) {
          throw std::logic_error(
              "This class assumes that the first non-world body is the "
              "floating body.");
        }

        if (body_ptr->get_position_start_index() != 0 ||
            body_ptr->get_velocity_start_index() != 0) {
          throw std::logic_error(
              "This class assumes that floating joint positions and are at the "
              "head of the position and velocity vectors.");
        }
      } else {
        if (joint.get_num_positions() > 1 || joint.get_num_velocities() > 1) {
          throw std::logic_error(
              "robot_state_t assumes non-floating joints to be "
              "1-DoF or fixed.");
        }
      }
    }
  }

  return robot;
}

bool RobotStateLcmMessageTranslator::CheckMessageVectorSize(
    const bot_core::robot_state_t& msg) {
  if (msg.num_joints < 0) return false;
  size_t size = static_cast<size_t>(msg.num_joints);
  if (size != msg.joint_name.size()) return false;
  if (size != msg.joint_position.size()) return false;
  if (size != msg.joint_velocity.size()) return false;
  if (size != msg.joint_effort.size()) return false;

  return true;
}

}  // namespace manipulation
}  // namespace drake

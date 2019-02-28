#include "drake/manipulation/util/robot_state_msg_translator.h"

#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/util/drakeGeometryUtil.h"

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
      root_body_(robot_.get_body(1)),
      is_floating_base_(root_body_.getJoint().is_floating()) {
  std::unordered_map<const RigidBody<double>*, std::string> body_to_joint_name;

  if (!is_floating_base_) {
    const DrakeJoint& root_joint = root_body_.getJoint();
    DRAKE_DEMAND(root_joint.is_fixed());
    DRAKE_DEMAND(root_joint.get_num_positions() == 0);
    DRAKE_DEMAND(root_joint.get_num_velocities() == 0);
  }

  for (const auto& body : robot.get_bodies()) {
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
  if (is_floating_base_) {
    const DrakeJoint& root_joint = root_body_.getJoint();

    // Pose of floating base in the world frame.
    const math::RigidTransform<double> X_WB(DecodePose(msg.pose));
    // Spatial velocity of base in the world frame.
    const Vector6<double> V_WB_W = DecodeTwist(msg.twist);
    // J is the root_joint frame.
    const math::RigidTransform<double> X_JW(
        root_joint.get_transform_to_parent_body().inverse());
    const math::RotationMatrix<double> R_JW = X_JW.rotation();
    // Reexpress V_WB_W in J frame.
    Vector6<double> V_WB_J;
    V_WB_J.head<3>() = R_JW * V_WB_W.head<3>();
    V_WB_J.tail<3>() = R_JW * V_WB_W.tail<3>();

    const math::RigidTransform<double> X_JB = X_JW * X_WB;
    const math::RotationMatrix<double> R_JB = X_JB.rotation();

    const DrakeJoint& floating_joint = root_body_.getJoint();
    const int position_start = root_body_.get_position_start_index();
    const int velocity_start = root_body_.get_velocity_start_index();

    if (floating_joint.get_num_positions() == 6) {
      // RPY-parameterized floating joint.
      // Translation.
      q.segment<3>(position_start) = X_JB.translation();

      // Orientation.
      const math::RollPitchYaw<double> rpy(R_JB);
      q.segment<3>(position_start + 3) = rpy.vector();

      // Translational velocity.
      auto translationdot = V_WB_J.tail<3>();
      v.segment<3>(velocity_start) = translationdot;

      // Rotational velocity.
      Eigen::Matrix<double, 3, 3> phi;
      typename math::Gradient<decltype(phi), Eigen::Dynamic>::type* dphi =
          nullptr;
      typename math::Gradient<decltype(phi), Eigen::Dynamic, 2>::type* ddphi =
          nullptr;
      angularvel2rpydotMatrix(rpy.vector(), phi, dphi, ddphi);
      auto angular_velocity_world = V_WB_J.head<3>();
      auto rpydot = (phi * angular_velocity_world).eval();
      v.segment<3>(velocity_start + 3) = rpydot;
    } else if (floating_joint.get_num_positions() == 7) {
      // Quaternion-parameterized floating joint.
      // Translation.
      q.segment<3>(position_start) = X_JB.translation();

      // Orientation.
      const Eigen::Vector4d quat = R_JB.ToQuaternionAsVector4();
      q.segment<4>(position_start + 3) = quat;

      // Express V_WB_J in the floating base's body frame (as V_WB_B).
      const math::RotationMatrix<double> R_BJ = R_JB.inverse();
      v.segment<3>(velocity_start) =  R_BJ * V_WB_J.head<3>();     // Rotate.
      v.segment<3>(velocity_start + 3) = R_BJ * V_WB_J.tail<3>();  // Translate.
    } else {
      throw std::domain_error(
          "Floating joint is neither a RPY or a Quaternion joint.");
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
  num_1dof_q -= root_body_.getJoint().get_num_positions();
  num_1dof_v -= root_body_.getJoint().get_num_velocities();
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
  const DrakeJoint& root_joint = root_body_.getJoint();
  // TODO(mitiguy) This code needs better notation and documentation.
  // To developers: Do not copy and paste this function's code elsewhere without
  // understanding it and how it works.  I am not this code's original author,
  // and find it difficult to understand.
  // J is the root_joint frame.
  math::RigidTransform<double> X_JB;  // Default is identity transform.
  Vector6<double> V_JB;

  if (is_floating_base_) {
    const int position_start = root_body_.get_position_start_index();
    const int velocity_start = root_body_.get_velocity_start_index();

    if (root_joint.get_num_positions() == 6) {
      // RPY
      const Vector3<double> position = q.segment<3>(position_start);
      const math::RollPitchYaw<double> rpy(q.segment<3>(position_start + 3));
      X_JB = math::RigidTransformd(rpy, position);

      Matrix3<double> phi = Matrix3<double>::Zero();
      angularvel2rpydotMatrix(rpy.vector(), phi,
                              static_cast<Matrix3<double>*>(nullptr),
                              static_cast<Matrix3<double>*>(nullptr));

      auto decomp = Eigen::ColPivHouseholderQR<Matrix3<double>>(phi);
      Vector3<double> rpydot = v.segment<3>(velocity_start + 3);
      V_JB.head<3>() = decomp.solve(rpydot);
      V_JB.tail<3>() = v.segment<3>(velocity_start);
    } else if (root_joint.get_num_positions() == 7) {
      // Quaternion
      const Vector3<double> position = q.segment<3>(position_start);
      const Vector4<double> wxyz = q.segment<4>(position_start + 3);
      const Eigen::Quaterniond quat(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
      const math::RotationMatrixd R_JB(quat);
      X_JB = math::RigidTransformd(R_JB, position);

      // Express velocity in the body frame.
      V_JB.head<3>() = R_JB * v.segment<3>(velocity_start);
      V_JB.tail<3>() = R_JB * v.segment<3>(velocity_start + 3);
    } else {
      throw std::domain_error(
          "Floating joint is neither a RPY or a Quaternion joint.");
    }
  } else {
    // Fixed base, the transformation is the joint's pose in the world frame.
    X_JB.SetIdentity();
    V_JB.setZero();
  }

  const math::RigidTransformd X_WJ(root_joint.get_transform_to_parent_body());
  const math::RigidTransformd X_WB = X_WJ * X_JB;
  Vector6<double> V_WB;
  V_WB.head<3>() = X_WJ.rotation() * V_JB.head<3>();
  V_WB.tail<3>() = X_WJ.rotation() * V_JB.tail<3>();

  EncodePose(X_WB.GetAsIsometry3(), msg->pose);
  EncodeTwist(V_WB, msg->twist);

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

  // Checks the "root_body".
  const RigidBody<double>& world = robot.get_body(0);
  const RigidBody<double>& root_body = robot.get_body(1);
  const DrakeJoint& root_joint = root_body.getJoint();

  DRAKE_DEMAND(root_body.has_parent_body());

  if (root_body.get_parent() != &world) {
    throw std::logic_error(
        "First body's parent must be the \"world\".");
  }

  bool floating_joint_found = false;
  bool fixed_base_found = false;

  if (root_joint.is_floating()) {
    floating_joint_found = true;

    if (root_body.get_position_start_index() != 0 ||
        root_body.get_velocity_start_index() != 0) {
      throw std::logic_error(
          "This class assumes that floating joint positions and velocities are "
          "at the head of the generalized position and velocity vectors.");
    }
  }

  if (root_joint.is_fixed()) {
    fixed_base_found = true;
    DRAKE_DEMAND(root_joint.get_num_positions() == 0);
    DRAKE_DEMAND(root_joint.get_num_velocities() == 0);
  }

  // Root body has to be either fixed or floating.
  if (!(floating_joint_found ^ fixed_base_found)) {
    throw std::logic_error(
        "The first body has to be attached to the world with either a fixed "
        "joint or a floating joint.");
  }

  // Skip the "world" and the "root_body".
  for (int i = 2; i < robot.get_num_bodies(); ++i) {
    const RigidBody<double>* body = robot.get_bodies()[i].get();
    DRAKE_DEMAND(body->has_parent_body());

    const auto& joint = body->getJoint();

    if (joint.is_floating()) {
      if (fixed_base_found) {
        throw std::logic_error(
            "This does not allow floating base and fixed base to co-exist.");
      }

      if (floating_joint_found) {
        throw std::logic_error(
            "robot_state_t assumes at most one floating joint.");
      }
    } else {
      if (joint.get_num_positions() > 1 || joint.get_num_velocities() > 1) {
        throw std::logic_error(
            "robot_state_t assumes non-floating joints to be "
            "1-DoF or fixed.");
      }

      if (joint.is_fixed()) {
        if (fixed_base_found && body->get_parent() == &world) {
          throw std::logic_error(
              "robot_state_t assumes at most one fixed root joint.");
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

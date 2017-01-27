#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/robotInterfaces/Side.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * An object that holds important kinematic properties.
 * For all the velocity / acceleration / wrench, the first 3 are always angular,
 * and the last 3 are linear.
 */
class BodyOfInterest {
 public:
  /**
   * @param name Name of this object. It does not have to match \p body's name.
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   * @param off Offset expressed in the body frame.
   */
  BodyOfInterest(const std::string& name, const RigidBody<double>& body,
                 const Vector3<double>& off)
      : name_(name), body_(&body), offset_(Eigen::Translation3d(off)) {}

  /**
   * Updates pose, velocity, Jacobian, Jacobian_dot_times_v based on @p robot
   * and @p cache.
   * @param robot is the robot model.
   * @param cache is the kinematics cache. It needs to be initialized first
   */
  void Update(const RigidBodyTree<double>& robot,
              const KinematicsCache<double>& cache) {
    pose_ = robot.CalcFramePoseInWorldFrame(cache, *body_, offset_);
    vel_ = robot.CalcFrameSpatialVelocityInWorldFrame(cache, *body_, offset_);
    J_ = robot.CalcFrameSpatialVelocityJacobianInWorldFrame(
        cache, *body_, offset_);
    Jdot_times_v_ = robot.CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
        cache, *body_, offset_);
  }

  inline const std::string& name() const { return name_; }
  inline const RigidBody<double>& body() const { return *body_; }
  inline const Isometry3<double>& pose() const { return pose_; }
  inline const Vector6<double>& velocity() const { return vel_; }
  inline const MatrixX<double>& J() const { return J_; }
  inline const Vector6<double>& Jdot_times_v() const { return Jdot_times_v_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Name of the BodyOfInterest
  std::string name_;
  // The link which this BOI is attached to
  const RigidBody<double>* body_;
  // Offset is specified in the body frame.
  Isometry3<double> offset_;

  Isometry3<double> pose_;
  // Spatial velocity in the world frame.
  Vector6<double> vel_;
  // Spatial velocity Jacobian, vel = J * v
  MatrixX<double> J_;
  // Spatial velocity Jacobian dot times v, Jdot * v
  Vector6<double> Jdot_times_v_;
};

/**
 * Mostly a thin wrapper on RigidBodyTree.
 * It has kinematic values such as spatial velocity of various body parts,
 * some measured contact force / torque information, joint torque, etc.
 * It also stores robot specific constants.
 */
class HumanoidStatus {
 public:
  /// Position offset from the foot frame to the sole frame.
  static const Vector3<double> kFootToSoleOffset;
  /// Position Offset from the foot frame to force torque sensor in the foot
  /// frame.
  static const Vector3<double> kFootToSensorPositionOffset;
  /// Rotation Offset from the foot frame to force torque sensor in the foot
  /// frame.
  static const Matrix3<double> kFootToSensorRotationOffset;

  /**
   * @param robot Reference to a RigidBodyTree, which must be valid through the
   * lifespan of this obejct.
   */
  HumanoidStatus(
      const RigidBodyTree<double>& robot,
      const param_parsers::RigidBodyTreeAliasGroups<double>& alias_group);

  /**
   * Do kinematics and compute useful information based on kinematics and
   * measured force torque information.
   * @param time is in seconds
   * @param q is the vector or generalized positions.
   * @param v is the vector of generalized velocities.
   * @param joint_torque is joint torque, should be in the same order as @p v,
   * not
   * in robot->actuators order
   * @param l_wrench is wrench measured in the sensor frame.
   * @param r_wrench is wrench measured in the sensor frame.
   */
  void Update(double t, const Eigen::Ref<const VectorX<double>>& q,
              const Eigen::Ref<const VectorX<double>>& v,
              const Eigen::Ref<const VectorX<double>>& joint_torque,
              const Eigen::Ref<const Vector6<double>>& l_wrench,
              const Eigen::Ref<const Vector6<double>>& r_wrench) {
    if (q.size() != position_.size() || v.size() != velocity_.size() ||
        joint_torque.size() != joint_torque_.size()) {
      throw std::runtime_error("robot state update dimension mismatch.");
    }
    time_ = t;
    position_ = q;
    velocity_ = v;
    joint_torque_ = joint_torque;
    foot_wrench_raw_[Side::LEFT] = l_wrench;
    foot_wrench_raw_[Side::RIGHT] = r_wrench;
    Update();
  }

  void Update();

  // Getters
  inline const RigidBodyTree<double>& robot() const { return *robot_; }
  inline const KinematicsCache<double>& cache() const { return cache_; }
  inline const std::unordered_map<std::string, int>& body_name_to_id() const {
    return body_name_to_id_;
  }
  inline const std::unordered_map<std::string, int>& name_to_position_index()
      const {
    return name_to_position_index_;
  }
  inline const std::unordered_map<std::string, int>& name_to_velocity_index()
      const {
    return name_to_velocity_index_;
  }
  inline const std::unordered_map<std::string, int>& actuator_name_to_id()
      const {
    return actuator_name_to_actuator_index_;
  }

  inline double time() const { return time_; }
  inline const VectorX<double>& position() const { return position_; }
  inline const VectorX<double>& velocity() const { return velocity_; }
  inline const VectorX<double>& joint_torque() const { return joint_torque_; }
  inline double position(int i) const { return position_[i]; }
  inline double velocity(int i) const { return velocity_[i]; }
  inline double joint_torque(int i) const { return joint_torque_[i]; }

  inline const MatrixX<double>& M() const { return M_; }
  inline const VectorX<double>& bias_term() const { return bias_term_; }
  inline const Vector3<double>& com() const { return com_; }
  inline const Vector3<double>& comd() const { return comd_; }
  inline const MatrixX<double>& J_com() const { return J_com_; }
  inline const Vector3<double>& Jdot_times_v_com() const {
    return Jdot_times_v_com_;
  }
  inline const MatrixX<double>& centroidal_momentum_matrix() const {
    return centroidal_momentum_matrix_;
  }
  inline const Vector6<double>& centroidal_momentum_matrix_dot_times_v() const {
    return centroidal_momentum_matrix_dot_times_v_;
  }
  inline const Vector6<double>& centroidal_momentum() const {
    return centroidal_momentum_;
  }
  inline const BodyOfInterest& pelvis() const {
    return bodies_of_interest_.at("pelvis");
  }
  inline const BodyOfInterest& torso() const {
    return bodies_of_interest_.at("torso");
  }
  inline const BodyOfInterest& foot(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return bodies_of_interest_.at("left_foot");
    else
      return bodies_of_interest_.at("right_foot");
  }
  inline const BodyOfInterest& foot(int s) const {
    return foot(Side::values.at(s));
  }
  inline const BodyOfInterest& foot_sensor(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return bodies_of_interest_.at("left_foot_sensor");
    else
      return bodies_of_interest_.at("right_foot_sensor");
  }
  inline const Vector2<double>& cop() const { return cop_; }
  inline const Vector2<double>& cop_in_sole_frame(Side::SideEnum s) const {
    return cop_in_sole_frame_[s];
  }
  inline const Vector6<double>& foot_wrench_in_sole_frame(
      Side::SideEnum s) const {
    return foot_wrench_in_sole_frame_[s];
  }
  inline const Vector6<double>& foot_wrench_raw(Side::SideEnum s) const {
    return foot_wrench_raw_[s];
  }
  inline const Vector6<double>& foot_wrench_in_world_frame(
      Side::SideEnum s) const {
    return foot_wrench_in_world_frame_[s];
  }
  inline const BodyOfInterest& foot_sensor(int s) const {
    return foot_sensor(Side::values.at(s));
  }
  inline const Vector2<double>& cop_in_sole_frame(int s) const {
    return cop_in_sole_frame(Side::values.at(s));
  }
  inline const Vector6<double>& foot_wrench_in_sole_frame(int s) const {
    return foot_wrench_in_sole_frame(Side::values.at(s));
  }
  inline const Vector6<double>& foot_wrench_raw(int s) const {
    return foot_wrench_raw(Side::values.at(s));
  }
  inline const Vector6<double>& foot_wrench_in_world_frame(int s) const {
    return foot_wrench_in_world_frame(Side::values.at(s));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBodyTree<double>* robot_;
  KinematicsCache<double> cache_;

  // Map body name to its index.
  std::unordered_map<std::string, int> body_name_to_id_;
  // Map position name to its index.
  std::unordered_map<std::string, int> name_to_position_index_;
  // Map velocity name to its index.
  std::unordered_map<std::string, int> name_to_velocity_index_;
  // Map actuator name to its index.
  std::unordered_map<std::string, int> actuator_name_to_actuator_index_;

  double time_;

  // Pos and Vel include 6 dof for the floating base.
  VectorX<double> position_;  /// Position in generalized coordinate
  VectorX<double> velocity_;  /// Velocity in generalized coordinate
  // In the same order as vel, but torque contains only actuated joints.
  VectorX<double> joint_torque_;  /// Joint torque

  MatrixX<double> M_;          ///< Inertial matrix
  VectorX<double> bias_term_;  ///< Bias term: M * vd + h = tau + J^T * lambda

  // Computed from kinematics
  Vector3<double> com_;               ///< Center of mass
  Vector3<double> comd_;              ///< Com velocity
  MatrixX<double> J_com_;             ///< Com Jacobian: comd = J_com * v
  Vector3<double> Jdot_times_v_com_;  ///< J_com_dot * v

  // Centroidal momentum = [angular; linear] momentum.
  // [angular; linear] = centroidal_momentum_matrix_ * v
  MatrixX<double> centroidal_momentum_matrix_;
  Vector6<double> centroidal_momentum_matrix_dot_times_v_;
  Vector6<double> centroidal_momentum_;

  // A list of body of interest, e.g. pelvis, feet, etc.
  eigen_aligned_std_unordered_map<std::string, BodyOfInterest>
      bodies_of_interest_;

  // Center of pressure
  Vector2<double> cop_;
  // Individual center of pressure in foot frame
  Vector2<double> cop_in_sole_frame_[2];

  // Wrench expressed in a frame that is aligned with the world frame, and
  // is located at the origin of the foot frame.
  Vector6<double> foot_wrench_in_world_frame_[2];

  // Wrench expressed in a frame that is aligned with the foot frame, and
  // is offsetted from the foot frame by kFootToSoleOffset.
  Vector6<double> foot_wrench_in_sole_frame_[2];

  // Untransformed raw measurement from the foot force torque sensor
  Vector6<double> foot_wrench_raw_[2];
};

std::ostream& operator<<(std::ostream& out, const HumanoidStatus& robot_status);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

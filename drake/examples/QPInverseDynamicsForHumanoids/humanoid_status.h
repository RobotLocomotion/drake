#pragma once

#include <set>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/rigid_body_tree_utils.h"
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
      : name_(name), body_(&body), offset_(off) {}

  /**
   * Updates pose, velocity, Jacobian, Jacobian_dot_times_v based on @p robot
   * and @p cache.
   * @param robot is the robot model.
   * @param cache is the kinematics cache. It needs to be initialized first
   */
  void Update(const RigidBodyTree<double>& robot,
              const KinematicsCache<double>& cache) {
    pose_.translation() = offset_;
    pose_.linear().setIdentity();
    pose_ = robot.relativeTransform(cache, 0, body_->get_body_index()) * pose_;

    vel_ = GetTaskSpaceVel(robot, cache, *body_, offset_);
    J_ = GetTaskSpaceJacobian(robot, cache, *body_, offset_);
    Jdot_times_v_ =
        GetTaskSpaceJacobianDotTimesV(robot, cache, *body_, offset_);
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
  Vector3<double> offset_;

  Isometry3<double> pose_;
  // This is the task space velocity, or twist of a frame that has the same
  // orientation as the world frame, but located at the origin of the body
  // frame.
  Vector6<double> vel_;
  // Task space Jacobian, xdot = J * v
  MatrixX<double> J_;
  // Task space Jd * v
  Vector6<double> Jdot_times_v_;
};

/**
 * Mostly a thin wrapper on RigidBodyTree.
 * It has kinematic values such as task space velocity of various body parts,
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
  explicit HumanoidStatus(const RigidBodyTree<double>& robot)
      : robot_(&robot),
        cache_(robot_->CreateKinematicsCache()),
        // TODO(siyuan.feng): The names of the links are hard coded for
        // Valkyrie, and they should be specified in some separate config file.
        bodies_of_interest_{
            BodyOfInterest("pelvis", *robot_->FindBody("pelvis"),
                           Vector3<double>::Zero()),
            BodyOfInterest("torso", *robot_->FindBody("torso"),
                           Vector3<double>::Zero()),
            BodyOfInterest("leftFoot", *robot_->FindBody("leftFoot"),
                           Vector3<double>::Zero()),
            BodyOfInterest("rightFoot", *robot_->FindBody("rightFoot"),
                           Vector3<double>::Zero()),
            BodyOfInterest("leftFootSensor", *robot_->FindBody("leftFoot"),
                           kFootToSensorPositionOffset),
            BodyOfInterest("rightFootSensor", *robot_->FindBody("rightFoot"),
                           kFootToSensorPositionOffset)} {
    time_ = 0;

    position_.resize(robot_->get_num_positions());
    velocity_.resize(robot_->get_num_velocities());
    joint_torque_.resize(robot_->actuators.size());
    nominal_position_ = robot_->getZeroConfiguration();

    // Build various lookup maps.
    body_name_to_id_ = std::unordered_map<std::string, int>();
    for (auto it = robot_->bodies.begin(); it != robot_->bodies.end(); ++it) {
      body_name_to_id_[(*it)->get_name()] = it - robot_->bodies.begin();
    }

    name_to_position_index_ = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot_->get_num_positions(); ++i) {
      name_to_position_index_[robot_->get_position_name(i)] = i;
    }
    name_to_velocity_index_ = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot_->get_num_velocities(); ++i) {
      name_to_velocity_index_[robot_->get_velocity_name(i)] = i;
    }
    for (int i = 0; i < static_cast<int>(robot_->actuators.size()); ++i) {
      actuator_name_to_actuator_index_[robot_->actuators.at(i).name_] = i;
    }

    // TODO(siyuan.feng): these are hard coded for Valkyrie, and they should
    // be included in the model file or loaded from a separate config file.
    nominal_position_[name_to_position_index().at("base_z")] = 1.025;
    nominal_position_[name_to_position_index().at("rightHipPitch")] = -0.49;
    nominal_position_[name_to_position_index().at("rightKneePitch")] = 1.205;
    nominal_position_[name_to_position_index().at("rightAnklePitch")] = -0.71;

    nominal_position_[name_to_position_index().at("leftHipPitch")] = -0.49;
    nominal_position_[name_to_position_index().at("leftKneePitch")] = 1.205;
    nominal_position_[name_to_position_index().at("leftAnklePitch")] = -0.71;

    nominal_position_[name_to_position_index().at("rightShoulderPitch")] =
        0.300196631343025;
    nominal_position_[name_to_position_index().at("rightShoulderRoll")] = 1.25;
    nominal_position_[name_to_position_index().at("rightElbowPitch")] =
        0.785398163397448;
    nominal_position_[name_to_position_index().at("rightForearmYaw")] = 1.571;

    nominal_position_[name_to_position_index().at("leftShoulderPitch")] =
        0.300196631343025;
    nominal_position_[name_to_position_index().at("leftShoulderRoll")] = -1.25;
    nominal_position_[name_to_position_index().at("leftElbowPitch")] =
      -0.785398163397448;
    nominal_position_[name_to_position_index().at("leftForearmYaw")] = 1.571;

    leg_joint_names_.insert("rightHipYaw");
    leg_joint_names_.insert("rightHipRoll");
    leg_joint_names_.insert("rightHipPitch");
    leg_joint_names_.insert("rightKneePitch");
    leg_joint_names_.insert("rightAnklePitch");
    leg_joint_names_.insert("rightAnkleRoll");
    leg_joint_names_.insert("leftHipYaw");
    leg_joint_names_.insert("leftHipRoll");
    leg_joint_names_.insert("leftHipPitch");
    leg_joint_names_.insert("leftKneePitch");
    leg_joint_names_.insert("leftAnklePitch");
    leg_joint_names_.insert("leftAnkleRoll");

    back_joint_names_.insert("torsoYaw");
    back_joint_names_.insert("torsoPitch");
    back_joint_names_.insert("torsoRoll");

    arm_joint_names_.insert("rightShoulderPitch");
    arm_joint_names_.insert("rightShoulderRoll");
    arm_joint_names_.insert("rightShoulderYaw");
    arm_joint_names_.insert("rightElbowPitch");
    arm_joint_names_.insert("rightForearmYaw");
    arm_joint_names_.insert("rightWristRoll");
    arm_joint_names_.insert("rightWristPitch");
    arm_joint_names_.insert("leftShoulderPitch");
    arm_joint_names_.insert("leftShoulderRoll");
    arm_joint_names_.insert("leftShoulderYaw");
    arm_joint_names_.insert("leftElbowPitch");
    arm_joint_names_.insert("leftForearmYaw");
    arm_joint_names_.insert("leftWristRoll");
    arm_joint_names_.insert("leftWristPitch");

    neck_joint_names_.insert("lowerNeckPitch");
  }

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

  /**
   * Returns a nominal q.
   */
  VectorX<double> GetNominalPosition() const { return nominal_position_; }

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
  inline const std::set<std::string>& leg_joint_names() const {
    return leg_joint_names_;
  }
  inline const std::set<std::string>& arm_joint_names() const {
    return arm_joint_names_;
  }
  inline const std::set<std::string>& back_joint_names() const {
    return back_joint_names_;
  }
  inline const std::set<std::string>& neck_joint_names() const {
    return neck_joint_names_;
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
  inline const BodyOfInterest& pelvis() const { return bodies_of_interest_[0]; }
  inline const BodyOfInterest& torso() const { return bodies_of_interest_[1]; }
  inline const BodyOfInterest& foot(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return bodies_of_interest_[2];
    else
      return bodies_of_interest_[3];
  }
  inline const BodyOfInterest& foot(int s) const {
    return foot(Side::values.at(s));
  }
  inline const BodyOfInterest& foot_sensor(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return bodies_of_interest_[4];
    else
      return bodies_of_interest_[5];
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

  // Nominal position for the robot.
  // TODO(siyuan.feng): should read this from the model file eventually.
  VectorX<double> nominal_position_;

  // Map body name to its index.
  std::unordered_map<std::string, int> body_name_to_id_;
  // Map position name to its index.
  std::unordered_map<std::string, int> name_to_position_index_;
  // Map velocity name to its index.
  std::unordered_map<std::string, int> name_to_velocity_index_;
  // Map actuator name to its index.
  std::unordered_map<std::string, int> actuator_name_to_actuator_index_;

  std::set<std::string> leg_joint_names_;
  std::set<std::string> arm_joint_names_;
  std::set<std::string> back_joint_names_;
  std::set<std::string> neck_joint_names_;

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
  std::vector<BodyOfInterest> bodies_of_interest_;

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

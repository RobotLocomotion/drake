#pragma once

#include <string>
#include <vector>

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
 private:
  /// Name of the BodyOfInterest
  std::string name_;
  /// The link which this BOI is attached to
  const RigidBody& body_;
  /// Offset is specified in the body frame.
  Eigen::Vector3d offset_;

  Eigen::Isometry3d pose_;
  /// This is the task space velocity, or twist of a frame that has the same
  /// orientation as the world frame, but located at the origin of the body
  /// frame.
  Eigen::Vector6d vel_;
  /// Task space Jacobian, xdot = J * v
  Eigen::MatrixXd J_;
  /// Task space Jd * v
  Eigen::Vector6d Jdot_times_v_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit BodyOfInterest(const std::string& name, const RigidBody& body,
                          const Eigen::Vector3d& off)
      : name_(name), body_(body), offset_(off) {}

  /**
   * Updates pose, velocity, Jacobian, Jacobian_dot_times_v based on @p robot
   * and @p cache.
   * @param robot is the robot model.
   * @param cache is the kinematics cache. It needs to be initialized first
   */
  void Update(const RigidBodyTree& robot,
              const KinematicsCache<double>& cache) {
    pose_.translation() = offset_;
    pose_.linear().setIdentity();
    pose_ = robot.relativeTransform(cache, 0, body_.get_body_index()) * pose_;

    vel_ = GetTaskSpaceVel(robot, cache, body_, offset_);
    J_ = GetTaskSpaceJacobian(robot, cache, body_, offset_);
    Jdot_times_v_ = GetTaskSpaceJacobianDotTimesV(robot, cache, body_, offset_);
  }

  inline const std::string& name() const { return name_; }
  inline const RigidBody& body() const { return body_; }
  inline const Eigen::Isometry3d& pose() const { return pose_; }
  inline const Eigen::Vector6d& velocity() const { return vel_; }
  inline const Eigen::MatrixXd& J() const { return J_; }
  inline const Eigen::Vector6d& Jdot_times_v() const { return Jdot_times_v_; }
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
  static const Eigen::Vector3d kFootToSoleOffset;
  /// Position Offset from the foot frame to force torque sensor in the foot
  /// frame.
  static const Eigen::Vector3d kFootToSensorPositionOffset;
  /// Rotation Offset from the foot frame to force torque sensor in the foot
  /// frame.
  static const Eigen::Matrix3d kFootToSensorRotationOffset;

  // TODO(siyuan.feng@tri.global): The names of the links are hard coded for
  // Valkyrie, and they should be specified in some separate config file.
  explicit HumanoidStatus(const RigidBodyTree& robot_in)
      : robot_(robot_in),
        cache_(robot_.bodies),
        bodies_of_interest_{
            BodyOfInterest("pelvis", *robot_.FindBody("pelvis"),
                           Eigen::Vector3d::Zero()),
            BodyOfInterest("torso", *robot_.FindBody("torso"),
                           Eigen::Vector3d::Zero()),
            BodyOfInterest("leftFoot", *robot_.FindBody("leftFoot"),
                           Eigen::Vector3d::Zero()),
            BodyOfInterest("rightFoot", *robot_.FindBody("rightFoot"),
                           Eigen::Vector3d::Zero()),
            BodyOfInterest("leftFootSensor", *robot_.FindBody("leftFoot"),
                           kFootToSensorPositionOffset),
            BodyOfInterest("rightFootSensor", *robot_.FindBody("rightFoot"),
                           kFootToSensorPositionOffset)} {
    time_ = 0;

    position_.resize(robot_.get_num_positions());
    velocity_.resize(robot_.get_num_velocities());
    joint_torque_.resize(robot_.actuators.size());
    nominal_position_ = robot_.getZeroConfiguration();

    // Build various lookup maps.
    body_name_to_id_ = std::unordered_map<std::string, int>();
    for (auto it = robot_.bodies.begin(); it != robot_.bodies.end(); ++it) {
      body_name_to_id_[(*it)->get_name()] = it - robot_.bodies.begin();
    }

    name_to_position_index_ = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot_.get_num_positions(); i++) {
      name_to_position_index_[robot_.get_position_name(i)] = i;
    }
    name_to_velocity_index_ = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot_.get_num_velocities(); i++) {
      name_to_velocity_index_[robot_.get_velocity_name(i)] = i;
    }
    for (int i = 0; i < static_cast<int>(robot_.actuators.size()); i++) {
      actuator_name_to_actuator_index_[robot_.actuators.at(i).name_] = i;
    }

    // TODO(siyuan.feng@tri.global): these are hard coded for Valkyrie, and they
    // should be included in the model file or loaded from a separate config
    // file.
    nominal_position_[name_to_position_index().at("rightHipRoll")] = 0.01;
    nominal_position_[name_to_position_index().at("rightHipPitch")] = -0.5432;
    nominal_position_[name_to_position_index().at("rightKneePitch")] = 1.2195;
    nominal_position_[name_to_position_index().at("rightAnklePitch")] = -0.7070;
    nominal_position_[name_to_position_index().at("rightAnkleRoll")] = -0.0069;

    nominal_position_[name_to_position_index().at("leftHipRoll")] = -0.01;
    nominal_position_[name_to_position_index().at("leftHipPitch")] = -0.5432;
    nominal_position_[name_to_position_index().at("leftKneePitch")] = 1.2195;
    nominal_position_[name_to_position_index().at("leftAnklePitch")] = -0.7070;
    nominal_position_[name_to_position_index().at("leftAnkleRoll")] = 0.0069;

    nominal_position_[name_to_position_index().at("rightShoulderRoll")] = 1;
    nominal_position_[name_to_position_index().at("rightShoulderYaw")] = 0.5;
    nominal_position_[name_to_position_index().at("rightElbowPitch")] =
        M_PI / 2.;

    nominal_position_[name_to_position_index().at("leftShoulderRoll")] = -1;
    nominal_position_[name_to_position_index().at("leftShoulderYaw")] = 0.5;
    nominal_position_[name_to_position_index().at("leftElbowPitch")] =
        -M_PI / 2.;
  }

  void UpdateFromOther(const HumanoidStatus& other) {
    if (&robot_ != &other.robot()) {
      throw std::runtime_error("can't assign with different robot ref");
    }

    if (&other == this) {
      return;
    }

    Update(other.time(), other.position(), other.velocity(),
           other.joint_torque(), other.foot_wrench_raw(Side::LEFT),
           other.foot_wrench_raw(Side::RIGHT));
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
  void Update(double t, const Eigen::Ref<const Eigen::VectorXd>& q,
              const Eigen::Ref<const Eigen::VectorXd>& v,
              const Eigen::Ref<const Eigen::VectorXd>& joint_torque,
              const Eigen::Ref<const Eigen::Vector6d>& l_wrench,
              const Eigen::Ref<const Eigen::Vector6d>& r_wrench);

  /**
   * Returns a nominal q.
   */
  Eigen::VectorXd GetNominalPosition() const { return nominal_position_; }

  // Getters
  inline const RigidBodyTree& robot() const { return robot_; }
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
    return name_to_position_index_;
  }
  inline const std::unordered_map<std::string, int>& actuator_name_to_id()
      const {
    return actuator_name_to_actuator_index_;
  }
  inline double time() const { return time_; }
  inline const Eigen::VectorXd& position() const { return position_; }
  inline const Eigen::VectorXd& velocity() const { return velocity_; }
  inline const Eigen::VectorXd& joint_torque() const { return joint_torque_; }
  inline const Eigen::MatrixXd& M() const { return M_; }
  inline const Eigen::VectorXd& bias_term() const { return bias_term_; }
  inline const Eigen::Vector3d& com() const { return com_; }
  inline const Eigen::Vector3d& comd() const { return comd_; }
  inline const Eigen::MatrixXd& J_com() const { return J_com_; }
  inline const Eigen::Vector3d& Jdot_times_v_com() const {
    return Jdot_times_v_com_;
  }
  inline const Eigen::MatrixXd& centroidal_momentum_matrix() const {
    return centroidal_momentum_matrix_;
  }
  inline const Eigen::Vector6d& centroidal_momentum_matrix_dot_times_v() const {
    return centroidal_momentum_matrix_dot_times_v_;
  }
  inline const Eigen::Vector6d& centroidal_momentum() const {
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
  inline const Eigen::Vector2d& cop() const { return cop_; }
  inline const Eigen::Vector2d& cop_in_sole_frame(Side::SideEnum s) const {
    return cop_in_sole_frame_[s];
  }
  inline const Eigen::Vector6d& foot_wrench_in_sole_frame(
      Side::SideEnum s) const {
    return foot_wrench_in_sole_frame_[s];
  }
  inline const Eigen::Vector6d& foot_wrench_raw(Side::SideEnum s) const {
    return foot_wrench_raw_[s];
  }
  inline const Eigen::Vector6d& foot_wrench_in_world_frame(
      Side::SideEnum s) const {
    return foot_wrench_in_world_frame_[s];
  }
  inline const BodyOfInterest& foot_sensor(int s) const {
    return foot_sensor(Side::values.at(s));
  }
  inline const Eigen::Vector2d& cop_in_sole_frame(int s) const {
    return cop_in_sole_frame(Side::values.at(s));
  }
  inline const Eigen::Vector6d& foot_wrench_in_sole_frame(int s) const {
    return foot_wrench_in_sole_frame(Side::values.at(s));
  }
  inline const Eigen::Vector6d& foot_wrench_raw(int s) const {
    return foot_wrench_raw(Side::values.at(s));
  }
  inline const Eigen::Vector6d& foot_wrench_in_world_frame(int s) const {
    return foot_wrench_in_world_frame(Side::values.at(s));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBodyTree& robot_;
  KinematicsCache<double> cache_;

  /// Nominal position for the robot.
  /// TODO(siyuan.feng@tri.global): should read this from the model file
  /// eventually.
  Eigen::VectorXd nominal_position_;

  /// Map body name to its index.
  std::unordered_map<std::string, int> body_name_to_id_;
  /// Map position name to its index.
  std::unordered_map<std::string, int> name_to_position_index_;
  /// Map velocity name to its index.
  std::unordered_map<std::string, int> name_to_velocity_index_;
  /// Map actuator name to its index.
  std::unordered_map<std::string, int> actuator_name_to_actuator_index_;

  double time_;

  // Pos and Vel include 6 dof for the floating base.
  Eigen::VectorXd position_;  /// Position in generalized coordinate
  Eigen::VectorXd velocity_;  /// Velocity in generalized coordinate
  // In the same order as vel, but torque contains only actuated joints.
  Eigen::VectorXd joint_torque_;  /// Joint torque

  Eigen::MatrixXd M_;          ///< Inertial matrix
  Eigen::VectorXd bias_term_;  ///< Bias term: M * vd + h = tau + J^T * lambda

  // Computed from kinematics
  Eigen::Vector3d com_;               ///< Center of mass
  Eigen::Vector3d comd_;              ///< Com velocity
  Eigen::MatrixXd J_com_;             ///< Com Jacobian: comd = J_com * v
  Eigen::Vector3d Jdot_times_v_com_;  ///< J_com_dot * v

  /// Centroidal momentum = [angular; linear] momentum.
  /// [angular; linear] = centroidal_momentum_matrix_ * v
  Eigen::MatrixXd centroidal_momentum_matrix_;
  Eigen::Vector6d centroidal_momentum_matrix_dot_times_v_;
  Eigen::Vector6d centroidal_momentum_;

  // A list of body of interest, e.g. pelvis, feet, etc.
  std::vector<BodyOfInterest> bodies_of_interest_;

  /// Center of pressure
  Eigen::Vector2d cop_;
  /// Individual center of pressure in foot frame
  Eigen::Vector2d cop_in_sole_frame_[2];

  /// Wrench expressed in a frame that is aligned with the world frame, and
  /// is located at the origin of the foot frame.
  Eigen::Vector6d foot_wrench_in_world_frame_[2];

  /// Wrench expressed in a frame that is aligned with the foot frame, and
  /// is offsetted from the foot frame by kFootToSoleOffset.
  Eigen::Vector6d foot_wrench_in_sole_frame_[2];

  /// Untransformed raw measurement from the foot force torque sensor
  Eigen::Vector6d foot_wrench_raw_[2];
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

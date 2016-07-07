#pragma once

#include "drake/systems/robotInterfaces/Side.h"
#include "rigid_body_tree_utils.h"

using namespace Eigen;

// For all the velocity / acceleration / wrench, the first 3 are always angular,
// and the last 3 are linear.

struct BodyOfInterest {
  std::string name;
  std::string link_name;
  Eigen::Isometry3d pose;
  // This is the task space velocity, or twist of a frame that has the same
  // orientation as the world frame, but located at the origin of the body
  // frame.
  Vector6d vel;

  // task space Jacobian, xdot = J * v
  MatrixXd J;
  // task space Jd * v
  Vector6d Jdot_times_v;
};

/**
 * Mostly a thin wrapper on RigidBodyTree.
 * It has kinematic values such as task space velocity of various body parts,
 * some measured contact force / torque information, joint torque, etc.
 */
class HumanoidStatus {
 public:
  /// Offset from the foot frame to contact position in the foot frame.
  static const Vector3d kFootToContactOffset;
  /// Offset from the foot frame to force torque sensor in the foot frame.
  static const Vector3d kFootToSensorOffset;

  std::unique_ptr<RigidBodyTree> robot;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> body_name_to_id;
  std::unordered_map<std::string, int> joint_name_to_id;
  std::unordered_map<std::string, int> actuator_name_to_id;

  explicit HumanoidStatus(std::unique_ptr<RigidBodyTree> robot_in)
      : robot(std::move(robot_in)),
        cache(robot->bodies) {
    pelv_.name = pelv_.link_name = std::string("pelvis");
    torso_.name = torso_.link_name = std::string("torso");
    foot_[Side::LEFT].name = foot_[Side::LEFT].link_name = std::string("leftFoot");
    foot_[Side::RIGHT].name = foot_[Side::RIGHT].link_name = std::string("rightFoot");
    foot_sensor_[Side::LEFT].name = std::string("leftFootSensor");
    foot_sensor_[Side::LEFT].link_name = std::string("leftFoot");
    foot_sensor_[Side::RIGHT].name = std::string("rightFootSensor");
    foot_sensor_[Side::RIGHT].link_name = std::string("rightFoot");

    // build map
    body_name_to_id = std::unordered_map<std::string, int>();
    for (auto it = robot->bodies.begin(); it != robot->bodies.end(); ++it) {
      body_name_to_id[(*it)->name()] = it - robot->bodies.begin();
    }

    joint_name_to_id = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot->number_of_positions(); i++) {
      joint_name_to_id[robot->getPositionName(i)] = i;
    }
    for (size_t i = 0; i < robot->actuators.size(); i++) {
      actuator_name_to_id[robot->actuators[i].name] = i;
    }

    time_ = time0_ = 0;

    position_.resize(robot->number_of_positions());
    velocity_.resize(robot->number_of_velocities());
    joint_torque_.resize(robot->actuators.size());
  }

  /**
   * @brief Do kinematics and compute useful information based on kinematics and
   * measured force torque information.
   *
   * @param t is in seconds
   * @param q is position
   * @param v is velocity
   * @param trq is joint torque, should be in the same order as @param v, not
   * in robot->actuators order
   * @param l_ft is force / torque measured at the foot force torque sensor
   * location.
   * @param r_ft is the same as @param l_ft
   * @param rot rotates @param l_ft and @param r_ft in the same orientation as
   * the foot frame. This is useful if the foot ft sensor has a different
   * orientation than the foot.
   */
  void Update(double t, const VectorXd& q, const VectorXd& v,
              const VectorXd& trq, const Vector6d& l_ft, const Vector6d& r_ft,
              const Matrix3d& rot = Matrix3d::Identity());

  inline double time() const { return time_; }
  inline const VectorXd &position() const { return position_; }
  inline const VectorXd &velocity() const { return velocity_; }
  inline const VectorXd &joint_torque() const { return joint_torque_; }
  inline const MatrixXd &M() const { return M_; }
  inline const VectorXd &bias_term() const { return bias_term_; }
  inline const Vector3d &com() const { return com_; }
  inline const Vector3d &comd() const { return comd_; }
  inline const MatrixXd &J_com() const { return J_com_; }
  inline const Vector3d &Jdot_times_v_com() const { return Jdot_times_v_com_; }
  inline const BodyOfInterest &pelv() const { return pelv_; }
  inline const BodyOfInterest &torso() const { return torso_; }
  inline const BodyOfInterest &foot(Side::SideEnum s) const { return foot_[s]; }
  inline const BodyOfInterest &foot(int s) const { return foot(Side::values.at(s)); }
  inline const BodyOfInterest &foot_sensor(Side::SideEnum s) const { return foot_sensor_[s]; }
  inline const Vector2d &cop() const { return cop_; }
  inline const Vector2d &cop_in_body_frame(Side::SideEnum s) const { return cop_in_body_frame_[s]; }
  inline const Vector6d &foot_wrench_in_body_frame(Side::SideEnum s) const { return foot_wrench_in_body_frame_[s]; }
  inline const Vector6d &foot_wrench_in_world_frame(Side::SideEnum s) const { return foot_wrench_in_world_frame_[s]; }
  
  inline const BodyOfInterest &foot_sensor(int s) const { return foot_sensor(Side::values.at(s)); }
  inline const Vector2d &cop_in_body_frame(int s) const { return cop_in_body_frame(Side::values.at(s)); }
  inline const Vector6d &foot_wrench_in_body_frame(int s) const { return foot_wrench_in_body_frame(Side::values.at(s)); }
  inline const Vector6d &foot_wrench_in_world_frame(int s) const { return foot_wrench_in_world_frame(Side::values.at(s)); }

 private:
  double time0_;
  double time_;

  // Pos and Vel include 6 dof for the floating base.
  VectorXd position_;  /// Position in generalized coordinate
  VectorXd velocity_;  /// Velocity in generalized coordinate
  // In the same order as vel, but trq contains only actuated joints.
  VectorXd joint_torque_;  /// Joint torque

  MatrixXd M_;          ///< Inertial matrix
  VectorXd bias_term_;  ///< Bias term: M * qdd + h = tau + J^T * lambda

  // computed from kinematics
  Vector3d com_;      ///< Center of mass
  Vector3d comd_;     ///< Com velocity
  MatrixXd J_com_;    ///< Com Jacobian: comd = J_com * v
  Vector3d Jdot_times_v_com_;  ///< J_com_dot * v

  // These are at the origin of the each body (defined by the urdf) unless
  // specified otherwise.
  BodyOfInterest pelv_;     ///< Pelvis link
  BodyOfInterest torso_;    ///< Torso
  BodyOfInterest foot_[2];  ///< At the bottom of foot, right below the ankle.
  BodyOfInterest foot_sensor_[2];  ///< At the foot sensor, inside foot

  Vector2d cop_;       ///< Center of pressure
  Vector2d cop_in_body_frame_[2];  ///< Individual center of pressure in foot frame

  Vector6d foot_wrench_in_body_frame_[2];  ///< Wrench measured in the body frame
  Vector6d foot_wrench_in_world_frame_[2];  ///< Wrench rotated to world frame

  /**
   * @brief computes kinematic related values
   *
   * @param name of BodyOfInterest
   * @param pose stores the output transformation
   * @param vel stores the output task space velocity
   * @param J stores the task space Jacobian
   * @param Jdot_times_v stores the task space Jacobian_dot * v
   * @param local_offset offset between point of interest to body origin in
   * body frame
   */
  void FillKinematics(const std::string& name, Isometry3d& pose, Vector6d& vel,
                      MatrixXd& J, Vector6d& Jdot_times_v,
                      const Vector3d& local_offset = Vector3d::Zero()) const;
};

#pragma once

#include "drake/systems/robotInterfaces/Side.h"
#include "rigid_body_tree_utils.h"

using namespace Eigen;

// For all the velocity / acceleration / wrench, the first 3 are always angular,
// and the last 3 are linear.
class BodyOfInterest {
 public:
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
  Vector6d Jdv;

  explicit BodyOfInterest(const std::string& n) {
    name = n;
    link_name = n;
  }

  BodyOfInterest(const std::string& n, const std::string& ln) {
    name = n;
    link_name = ln;
  }
};

/**
 * Mostly a thin wrapper on RigidBodyTree.
 * It has kinematic values such as task space velocity of various body parts,
 * some measured contact force / torque information, joint torque, etc.
 */
class HumanoidState {
 public:
  ///< Offset from the foot frame to contact position in the foot frame.
  static const Vector3d kFootToContactOffset;
  ///< Offset from the foot frame to force torque sensor in the foot frame.
  static const Vector3d kFootToSensorOffset;

  double time;

  std::unique_ptr<RigidBodyTree> robot;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> body_name_to_id;
  std::unordered_map<std::string, int> joint_name_to_id;
  std::unordered_map<std::string, int> actuator_name_to_id;

  // Pos and Vel include 6 dof for the floating base.
  VectorXd pos;  ///< Position in generalized coordinate
  VectorXd vel;  ///< Velocity in generalized coordinate
  // In the same order as vel, but trq contains only actuated joints.
  VectorXd trq;  ///< Joint torque

  MatrixXd M;  ///< Inertial matrix
  VectorXd h;  ///< Bias term: M * qdd + h = tau + J^T * lambda

  // computed from kinematics
  Vector3d com;      ///< Center of mass
  Vector3d comd;     ///< Com velocity
  MatrixXd J_com;    ///< Com Jacobian: comd = J_com * v
  Vector3d Jdv_com;  ///< J_com_dot * v

  // These are at the origin of the each body (defined by the urdf) unless
  // specified otherwise.
  BodyOfInterest pelv;    ///< Pelvis link
  BodyOfInterest torso;   ///< Torso
  BodyOfInterest l_foot;  ///< At the bottom of foot, right below the ankle.
  BodyOfInterest r_foot;
  BodyOfInterest l_foot_sensor;  ///< At the foot sensor, inside foot
  BodyOfInterest r_foot_sensor;

  // easy access to l_foot, r_foot
  BodyOfInterest* foot[2];
  BodyOfInterest* foot_sensor[2];

  Vector2d cop;       ///< Center of pressure
  Vector2d cop_b[2];  ///< Individual center of pressure in foot frame

  Vector6d footFT_b[2];  ///< Wrench measured in the body frame
  Vector6d footFT_w[2];  ///< Wrench rotated to world frame

  explicit HumanoidState(std::unique_ptr<RigidBodyTree> robot_in)
      : robot(std::move(robot_in)),
        cache(robot->bodies),
        pelv("pelvis"),
        torso("torso"),
        l_foot("leftFoot"),
        r_foot("rightFoot"),
        l_foot_sensor("leftFootSensor", "leftFoot"),
        r_foot_sensor("rightFootSensor", "rightFoot") {
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

    foot[Side::LEFT] = &l_foot;
    foot[Side::RIGHT] = &r_foot;

    foot_sensor[Side::LEFT] = &l_foot_sensor;
    foot_sensor[Side::RIGHT] = &r_foot_sensor;

    time = time0_ = 0;

    pos.resize(robot->number_of_positions());
    vel.resize(robot->number_of_velocities());
    trq.resize(robot->actuators.size());
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

 private:
  double time0_;

  /**
   * @brief computes kinematic related values
   *
   * @param name of BodyOfInterest
   * @param pose stores the output transformation
   * @param vel stores the output task space velocity
   * @param J stores the task space Jacobian
   * @param Jdv stores the task space Jacobian_dot * v
   * @param local_offset offset between point of interest to body origin in
   * body frame
   */
  void FillKinematics(const std::string& name, Isometry3d& pose, Vector6d& vel,
                      MatrixXd& J, Vector6d& Jdv,
                      const Vector3d& local_offset = Vector3d::Zero()) const;
};

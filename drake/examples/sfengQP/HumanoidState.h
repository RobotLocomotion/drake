#pragma once

#include "drake/systems/robotInterfaces/Side.h"
#include "sfUtils.h"

using namespace Eigen;

class BodyOfInterest {
 public:
  std::string name;
  std::string link_name;
  Eigen::Isometry3d pose;
  // task space velocity, or twist of a frame that has the same orientation
  // as the world frame, but located at the origin of the body frame
  Vector6d vel;

  // task space Jacobian, xdot = J * v
  MatrixXd J;
  // task space Jd * v
  Vector6d Jdv;

  explicit BodyOfInterest(const std::string &n) {
    name = n;
    link_name = n;
  }

  BodyOfInterest(const std::string &n, const std::string &ln) {
    name = n;
    link_name = ln;
  }
};


class HumanoidState {
 public:
  double time;

  std::unique_ptr<RigidBodyTree> robot;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> bodyName2ID;
  std::unordered_map<std::string, int> jointName2ID;
  std::unordered_map<std::string, int> actuatorName2ID;

  // these have base 6
  VectorXd pos;
  VectorXd vel;
  VectorXd trq;  // in the same order as vel, but only has actuated joints

  MatrixXd M;  // inertial matrix
  VectorXd h;  // bias term: M * qdd + h = tau + J^T * lambda

  // computed from kinematics
  Vector3d com;  // center of mass
  Vector3d comd;  // com velocity
  MatrixXd J_com;  // com Jacobian: comd = J_com * v
  Vector3d Jdv_com;  // J_com_dot * v

  BodyOfInterest pelv;  // pelvis
  BodyOfInterest torso;  // torso
  BodyOfInterest l_foot;  // at the bottom of foot
  BodyOfInterest r_foot;
  BodyOfInterest l_foot_sensor;  // at the foot sensor
  BodyOfInterest r_foot_sensor;

  // easy access to l_foot, r_foot
  BodyOfInterest *foot[2];
  BodyOfInterest *foot_sensor[2];

  Vector2d cop;  // center of pressure
  Vector2d cop_b[2];  // individual center of pressue in foot frame

  Vector6d footFT_b[2];  // wrench measured in the body frame
  Vector6d footFT_w[2];  // wrench rotated to world frame

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
      bodyName2ID = std::unordered_map<std::string, int>();
      for (auto it = robot->bodies.begin(); it != robot->bodies.end(); ++it) {
        bodyName2ID[(*it)->name()] = it - robot->bodies.begin();
      }
      for (auto it = robot->frames.begin(); it != robot->frames.end(); ++it) {
        bodyName2ID[(*it)->name] = -(it - robot->frames.begin()) - 2;
      }

      jointName2ID = std::unordered_map<std::string, int>();
      for (int i = 0; i < robot->number_of_positions(); i++) {
        jointName2ID[robot->getPositionName(i)] = i;
      }
      for (size_t i = 0; i < robot->actuators.size(); i++) {
        actuatorName2ID[robot->actuators[i].name] = i;
      }

      foot[Side::LEFT] = &l_foot;
      foot[Side::RIGHT] = &r_foot;

      foot_sensor[Side::LEFT] = &l_foot_sensor;
      foot_sensor[Side::RIGHT] = &r_foot_sensor;

      time = _time0 = 0;

      pos.resize(robot->number_of_positions());
      vel.resize(robot->number_of_velocities());
      trq.resize(robot->actuators.size());
    }

  void update(
      double t,
      const VectorXd &q,
      const VectorXd &v,
      const VectorXd &trq,
      const Vector6d &l_ft,
      const Vector6d &r_ft,
      const Matrix3d &rot = Matrix3d::Identity());

 private:
  double _time0;

  void _fillKinematics(
      const std::string &name,
      Isometry3d &pose,
      Vector6d &vel,
      MatrixXd &J,
      Vector6d &Jdv,
      const Vector3d &local_offset = Vector3d::Zero());
};

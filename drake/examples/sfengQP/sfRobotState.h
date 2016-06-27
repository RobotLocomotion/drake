#pragma once

#include "drake/systems/robotInterfaces/Side.h"
#include "MRDLogger.h"
#include "sfUtils.h"

using namespace Eigen;

class BodyOfInterest {
public:
  std::string name;
  std::string link_name;
  Eigen::Isometry3d pose;
  Vector6d vel;
  
  MatrixXd J;
  Vector6d Jdv;

  BodyOfInterest(const std::string &n) 
  { 
    name = n;
    link_name = n;
  }
  
  BodyOfInterest(const std::string &n, const std::string &ln) 
  { 
    name = n;
    link_name = ln;
  }
  
  void addToLog(MRDLogger &logger) const
  {
    logger.addChannel(name+"[x]", "m", pose.translation().data());
    logger.addChannel(name+"[y]", "m", pose.translation().data()+1);
    logger.addChannel(name+"[z]", "m", pose.translation().data()+2);
    
    logger.addChannel(name+"d[x]", "m/s", vel.data()+3);
    logger.addChannel(name+"d[y]", "m/s", vel.data()+4);
    logger.addChannel(name+"d[z]", "m/s", vel.data()+5);
    logger.addChannel(name+"d[wx]", "m/s", vel.data()+0);
    logger.addChannel(name+"d[wy]", "m/s", vel.data()+1);
    logger.addChannel(name+"d[wz]", "m/s", vel.data()+2);
  }
};


class sfRobotState {
public:
  double time;
  
  std::unique_ptr<RigidBodyTree> robot;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> bodyName2ID;
  std::unordered_map<std::string, int> jointName2ID;
  
  // these have base 6
  VectorXd pos;
  VectorXd vel;
  // this only has actuated
  VectorXd trq;
  Vector6d footFT_b[2];

  MatrixXd M;
  VectorXd h;

  // computed from kinematics  
  Vector3d com;
  Vector3d comd;
  MatrixXd J_com;
  Vector3d Jdv_com;

  BodyOfInterest pelv;
  BodyOfInterest l_foot; // at the bottom of foot
  BodyOfInterest r_foot; // at the bottom of foot
  BodyOfInterest l_foot_sensor;
  BodyOfInterest r_foot_sensor;
  BodyOfInterest torso;

  std::vector<BodyOfInterest *> foot; // easy access to l_foot, r_foot
  std::vector<BodyOfInterest *> foot_sensor; // easy access to l_foot_sensor, r_foot_sensor

  Vector2d cop;
  Vector2d cop_b[2];
  
  Vector6d footFT_w[2];
  //Vector6d footFT_w_statics[2];
  
  sfRobotState(std::unique_ptr<RigidBodyTree> robot_in)
    : robot(std::move(robot_in)),
      cache(robot->bodies),
      pelv("pelvis"),
      l_foot("leftFoot"),
      r_foot("rightFoot"),
      l_foot_sensor("leftFootSensor", "leftFoot"),
      r_foot_sensor("rightFootSensor", "rightFoot"),
      torso("torso")
  {
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

    foot.resize(2);
    foot[Side::LEFT] = &l_foot;
    foot[Side::RIGHT] = &r_foot;
    
    foot_sensor.resize(2);
    foot_sensor[Side::LEFT] = &l_foot_sensor;
    foot_sensor[Side::RIGHT] = &r_foot_sensor;

    time = _time0 = 0;
  }

  //void parseMsg(const bot_core::robot_state_t &msg);
  void addToLog(MRDLogger &logger) const;

  // ft_l, and ft_r needs to be ROTATED FIRST s.t. x fwd, z up!!!
  void update(double t, const VectorXd &q, const VectorXd &v, const VectorXd &trq, const Vector6d &l_ft, const Vector6d &r_ft);

private:
  double _time0;

  void _fillKinematics(const std::string &name, Isometry3d &pose, Vector6d &vel, MatrixXd &J, Vector6d &Jdv, const Vector3d &local_offset = Vector3d::Zero());
};


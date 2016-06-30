#pragma once

#include "HumanoidState.h"
#include <iostream>
#include <fstream>

class QPInput {
 public:
  std::vector<std::string> jointNames;

  Vector3d comdd_d;
  Vector6d pelvdd_d;
  Vector6d torsodd_d;
  Vector6d footdd_d[2];
  VectorXd qdd_d;
  Vector6d wrench_d[2];

  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_hand;
  double w_qdd;
  double w_wrench_reg;

  QPInput() { _inited = false; }

  explicit QPInput(const RigidBodyTree& r) {
    init(r);
    qdd_d.resize(r.number_of_velocities());
    _inited = true;
  }

  bool isSane() const {
    bool ret = _inited;
    ret &= jointNames.size() == qdd_d.size();
    return ret;
  }

  void init(const RigidBodyTree& r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      jointNames.push_back(r.getPositionName(i));
    _inited = true;
  }

 private:
  bool _inited;
};

class QPOutput {
 public:
  std::vector<std::string> jointNames;

  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d torsodd;
  Vector6d footdd[2];
  VectorXd qdd;
  VectorXd trq;

  Vector6d foot_wrench_w[2];
  Vector6d foot_wrench_in_sensor_frame[2];

  QPOutput() { _inited = false; }

  explicit QPOutput(const RigidBodyTree& r) {
    init(r);
    _inited = true;
  }

  void init(const RigidBodyTree& r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      jointNames.push_back(r.getPositionName(i));
    _inited = true;
  }

  bool isSane() const {
    bool ret = _inited;
    ret &= jointNames.size() == qdd.size();
    ret &= (qdd.size() == trq.size()) || (qdd.size() == trq.size() + 6);
    return ret;
  }

  void print() const;
  double computeCost(const HumanoidState& rs, const QPInput& input) const;

 private:
  bool _inited;
};

class QPParam {
 public:
  double mu;     // Fx, Fy < |mu * Fz|
  double mu_Mz;  // Mz < |mu * Mz|
  double x_max;
  double x_min;
  double y_max;
  double y_min;

  QPParam() {
    mu = 1;
    mu_Mz = 0.1;
    x_max = 0.2;
    x_min = -0.05;
    y_max = 0.05;
    y_min = -0.05;
  }
};

class QPController {
 public:
  QPParam param;

  int control(const HumanoidState& rs, const QPInput& input, QPOutput& output);
};

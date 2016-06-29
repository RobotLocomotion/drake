#pragma once

#include "sfConfig.h"
#include "HumanoidState.h"
#include "sfQP.h"
#include <iostream>
#include <fstream>

class QPInput : public Configurable {
 public:
  std::vector<std::string> jointNames;

  Vector3d comdd_d;
  Vector6d pelvdd_d;
  Vector6d torsodd_d;
  Vector6d footdd_d[2];
  VectorXd qdd_d;

  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_hand;
  double w_qdd;
  double w_wrench_reg;

  QPInput() {
    _inited = false;
    _setupParamLookup();
  }

  explicit QPInput(const RigidBodyTree &r) {
    init(r);
    qdd_d.resize(r.number_of_velocities());
    _inited = true;
    _setupParamLookup();
  }

  bool isSane() const {
    bool ret = _inited;
    ret &= jointNames.size() == qdd_d.size();
    return ret;
  }

  void init(const RigidBodyTree &r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      jointNames.push_back(r.getPositionName(i));
    _inited = true;
  }

 protected:
  bool _setupParamLookup() {
    _paramLookup["w_com"] = &w_com;
    _paramLookup["w_pelv"] = &w_pelv;
    _paramLookup["w_torso"] = &w_torso;
    _paramLookup["w_foot"] = &w_foot;
    _paramLookup["w_hand"] = &w_hand;
    _paramLookup["w_qdd"] = &w_qdd;
    _paramLookup["w_wrench_reg"] = &w_wrench_reg;
    return true;
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

  QPOutput() {
    _inited = false;
  }

  explicit QPOutput(const RigidBodyTree &r) {
    init(r);
    _inited = true;
  }

  void init(const RigidBodyTree &r) {
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
  

 private:
  bool _inited;
};
  
inline bool operator== (const QPOutput &a, const QPOutput &b) {
  double e = 1e-10;
  bool ret = a.comdd.isApprox(b.comdd, e);
  std::cout << ret << std::endl;
  ret &= a.pelvdd.isApprox(b.pelvdd, e);
  std::cout << ret << std::endl;
  ret &= a.torsodd.isApprox(b.torsodd, e);
  std::cout << ret << std::endl;
  for (int i = 0; i < 2; i++) {
    ret &= a.footdd[i].isApprox(b.footdd[i], e);
    std::cout << ret << std::endl;
  }
  ret &= a.qdd.isApprox(b.qdd, e);
  std::cout << ret << std::endl;
  ret &= a.trq.isApprox(b.trq, e);
  std::cout << ret << std::endl;
  for (int i = 0; i < 2; i++) {
    ret &= a.foot_wrench_w[i].isApprox(b.foot_wrench_w[i], e);
  std::cout << ret << std::endl;
    ret &= a.foot_wrench_in_sensor_frame[i].isApprox(b.foot_wrench_in_sensor_frame[i], e);
  std::cout << ret << std::endl;
  }

  return ret;
}

class QPParam {
 public:
  double mu;  // Fx, Fy < |mu * Fz|
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

// this does it the old fashion
class QPController : public sfQP {
 public:
  QPParam param;

  int control(const HumanoidState &rs, const QPInput &input, QPOutput &output);
};

// really uses Optimization.h
class QPController2 {
 public:
  QPParam param;

  int control(const HumanoidState &rs, const QPInput &input, QPOutput &output); 
}; 

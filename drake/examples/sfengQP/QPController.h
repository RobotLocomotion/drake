#pragma once

#include "sfRobotState.h"

class QPInput {
public:
  std::vector<std::string> names;
  
  Vector3d comdd_d;
  Vector6d pelvdd_d;
  Vector6d torsodd_d;
  Vector6d footdd_d[2];
  VectorXd qdd_d;

  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_qdd;
  
  void init(const RigidBodyTree &r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      names.push_back(r.getPositionName(i));
    _inited = true;
  }

  QPInput() { _inited = false; }
  QPInput(const RigidBodyTree &r) { init(r); _inited = true; }
  
  bool isSane() const
  {
    bool ret = _inited;
    ret &= names.size() == qdd_d.size();
    return ret;
  }

private:
  bool _inited;
};

class QPOutput {
public:
  std::vector<std::string> names;

  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d torsodd;
  Vector6d footdd[2];
  VectorXd qdd;
  VectorXd trq;

  Vector6d foot_wrench_w[2];
  Vector6d foot_wrench_b[2];
  
  QPOutput() { _inited = false; }
  QPOutput(const RigidBodyTree &r) { init(r); _inited = true; }

  void init(const RigidBodyTree &r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      names.push_back(r.getPositionName(i));
    _inited = true;
  }
  
  bool isSane() const
  {
    bool ret = _inited;
    ret &= names.size() == qdd.size();
    ret &= (qdd.size() == trq.size()) || (qdd.size() == trq.size() + 6);
    return ret;
  }
  
  void print() const;

private:
  bool _inited;
};

class QPController {
public:
  int control(const sfRobotState &rs, const QPInput &input, QPOutput &output);

private:
  MatrixXd _CE, _CI, _H;
  VectorXd _ce0, _ci_l, _ci_u, _h0, _X;
};

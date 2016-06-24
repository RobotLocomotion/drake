#pragma once

#include "sfRobotState.h"
#include "sfQP.h"
#include <exception>
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

  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_hand;
  double w_qdd;
  double w_wrench_reg;
  
  void init(const RigidBodyTree &r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      jointNames.push_back(r.getPositionName(i));
    _inited = true;
  }

  QPInput() 
  { 
    _inited = false;
    _paramLookup["w_com"] = &w_com;
    _paramLookup["w_pelv"] = &w_pelv;
    _paramLookup["w_torso"] = &w_torso;
    _paramLookup["w_foot"] = &w_foot;
    _paramLookup["w_hand"] = &w_hand;
    _paramLookup["w_qdd"] = &w_qdd;
    _paramLookup["w_wrench_reg"] = &w_wrench_reg;
  }

  QPInput(const RigidBodyTree &r) 
  { 
    init(r);
    qdd_d.resize(r.number_of_velocities());
    _inited = true; 
  }
  
  bool isSane() const 
  {
    bool ret = _inited;
    ret &= jointNames.size() == qdd_d.size();
    return ret;
  }

  bool loadParamFromFile(const std::string &fileName) 
  {
    std::ifstream in(fileName);

    if (!in.good())
      return false;

    std::string name;
    std::map<const std::string, double *>::iterator res;
    double val;
    bool ret = true;
    while (true)
    {
      in >> name;
      if (in.eof())
        break;

      res = _paramLookup.find(name);
      // can't find item
      if (res == _paramLookup.end()) {
        std::cerr << "unknown param: " << name << " aborting." << std::endl;
        ret = false;
        break;
      }

      in >> val;
      *(res->second) = val;
      std::cerr << "read " << name << " = " << val << std::endl;
    }

    //printParams();

    return ret;
  }

  class QPInputException : public std::exception {
    virtual const char *what() const throw() {
      return "Invalid QP Input.";
    } 
  };

private:
  bool _inited;
  std::map<const std::string, double *> _paramLookup;

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
  QPOutput(const RigidBodyTree &r) { init(r); _inited = true; }

  void init(const RigidBodyTree &r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      jointNames.push_back(r.getPositionName(i));
    _inited = true;
  }
  
  bool isSane() const
  {
    bool ret = _inited;
    ret &= jointNames.size() == qdd.size();
    ret &= (qdd.size() == trq.size()) || (qdd.size() == trq.size() + 6);
    return ret;
  }
  
  void print() const;

  class QPOutputException : public std::exception {
    virtual const char *what() const throw() {
      return "Invalid QP Output.";
    } 
  };
 
private:
  bool _inited;
};

class QPParam {
public: 
  /*
  Vector6d Kp_pelv;
  Vector6d Kd_pelv; 
  
  Vector6d Kp_torso;
  Vector6d Kd_torso;
  
  Vector6d Kp_foot;
  Vector6d Kd_foot;
  
  Vector6d Kp_hand;
  Vector6d Kd_hand;
  */

  double mu; // Fx, Fy < |mu * Fz|
  double mu_Mz; // Mz < |mu * Mz|
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

class QPController : public sfQP {
public:
  QPParam param;

  int control(const sfRobotState &rs, const QPInput &input, QPOutput &output);

};

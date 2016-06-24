#pragma once

#include "sfRobotState.h"
#include "sfQP.h"
#include "sfConfig.h"
#include <exception>

class QPEstimator : public sfQP, public Configurable {
public:
  sfRobotState rs;
  double dt;
  
  VectorXd residual;
  VectorXd vel;
  VectorXd trq;
  VectorXd wrench;
  
  QPEstimator(const std::string &urdf) 
    : rs(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)))
  {
    _inited = false;
    _setupParamLookup();
    dt = 2e-3;
  }
      
  // ft_l, and ft_r needs to be ROTATED FIRST s.t. x fwd, z up!!!
  int estimate(double t, const VectorXd &q, const VectorXd &v, const VectorXd &trq, const Vector6d &ft_l, const Vector6d &ft_r);
  // ft_l, and ft_r needs to be ROTATED FIRST s.t. x fwd, z up!!!
  int init(double t, const VectorXd &q, const VectorXd &v, const VectorXd &trq, const Vector6d &ft_l, const Vector6d &ft_r);
  
  double w_error;
  double w_measured_vel;
  double w_measured_wrench;
  double w_measured_trq;

protected:
  bool _setupParamLookup()
  {
    _paramLookup["w_error"] = &w_error;
    _paramLookup["w_measured_vel"] = &w_measured_vel;
    _paramLookup["w_measured_wrench"] = &w_measured_wrench;
    _paramLookup["w_measured_trq"] = &w_measured_trq;
    return true;
  }

private:
  bool _inited;
}; 

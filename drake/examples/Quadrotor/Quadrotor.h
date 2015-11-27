#ifndef _QUADROTOR_H_
#define _QUADROTOR_H_

#include <iostream>
#include <cmath>
#include "DrakeSystem.h"
#include "LCMCoordinateFrame.h"
#include "BotVisualizer.h"

class Quadrotor : public DrakeSystem {
public:
  Quadrotor(const std::shared_ptr<lcm::LCM>& lcm) :
          DrakeSystem("Quadrotor",
                  std::make_shared<CoordinateFrame>("QuadrotorContState", std::vector<std::string>({"x", "y", "z", "roll", "pitch", "yaw", "xdot", "ydot", "zdot", "rolldot", "pitchdot", "yawdot"})),
                  nullptr,
                  std::make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("QuadrotorInput", std::vector<std::string>({"u1, u2, u3, u4"}), lcm),
                  std::make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("QuadrotorState", std::vector<std::string>({"x", "y", "z", "roll", "pitch", "yaw", "xdot", "ydot", "zdot", "rolldot", "pitchdot", "yawdot"}), lcm)),
            m(1.0),
            g(9.81),
            L(0.1750)
  {}
  virtual ~Quadrotor(void) {};

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return dynamics_implementation(t,x,u);
  }

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return x;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return false; }


  DrakeSystemPtr balanceLQR() {
    Eigen::MatrixXd Q(12,12);  Q = Eigen::MatrixXd::Identity(12,12);
    Eigen::MatrixXd R(4,1);  R << 1;
    Eigen::VectorXd xG(12);   xG << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd uG(4);   uG = Eigen::VectorXd::Ones(4) * m * g * 0.25;

    return timeInvariantLQR(xG,uG,Q,R);
  }

  double g, L, I, m;

private:
  template <typename Scalar, typename Vector>
  Vector dynamics_implementation(Scalar t, const Vector& x, const Vector& u) const {
    Vector xdot(12);
    xdot = Vector::Zero(12);
    return xdot;
  }

};

class QuadrotorWithBotVis : public Quadrotor {
public:
  QuadrotorWithBotVis(const std::shared_ptr<lcm::LCM>& lcm) 
  : Quadrotor(lcm), botvis(lcm, "Quadrotor.urdf", DrakeJoint::ROLLPITCHYAW) {}

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    botvis.output(t,Eigen::VectorXd::Zero(0),x);
    return Quadrotor::output(t,x,u);
  }

  BotVisualizer botvis;
};

#endif // _QUADROTOR_H_

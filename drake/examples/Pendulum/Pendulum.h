#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <iostream>
#include <cmath>
#include "DrakeSystem.h"
#include "LCMCoordinateFrame.h"
#include "BotVisualizer.h"

class Pendulum : public DrakeSystem {
public:
  Pendulum(const std::shared_ptr<lcm::LCM>& lcm) :
          DrakeSystem("Pendulum",
                  std::make_shared<CoordinateFrame>("PendulumContState",std::vector<std::string>({"theta","thetadot"})),
                  nullptr,
                  std::make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumInput",std::vector<std::string>({"tau"}),lcm),
                  std::make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumState",std::vector<std::string>({"theta","thetadot"}),lcm)),
          m(1.0), // kg
          l(.5),  // m
          b(0.1), // kg m^2 /s
          lc(.5), // m
          I(.25), // m*l^2; % kg*m^2
          g(9.81) // m/s^2
  {}
  virtual ~Pendulum(void) {};

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return dynamics_implementation(t,x,u);
  }
  virtual Drake::TaylorVecX dynamics(Drake::TaylorVarX t, const Drake::TaylorVecX& x, const Drake::TaylorVecX& u) const override {
    return dynamics_implementation(t,x,u);
  }

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return x;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return false; }


  DrakeSystemPtr balanceLQR() {
    Eigen::MatrixXd Q(2,2);  Q << 10, 0, 0, 1;
    Eigen::MatrixXd R(1,1);  R << 1;
    Eigen::VectorXd xG(2);   xG << M_PI, 0;
    Eigen::VectorXd uG(1);   uG << 0;

    return timeInvariantLQR(xG,uG,Q,R);
  }


  double m,l,b,lc,I,g;  // pendulum parameters (initialized in the constructor)

private:
  template <typename Scalar,typename Vector>
  Vector dynamics_implementation(Scalar t, const Vector& x, const Vector& u) const {
    Vector xdot(2);
    xdot(0) = x(1);
    xdot(1) = (u(0) - m*g*lc*sin(x(0)) - b*x(1))/I;
    return xdot;
  }

};

class PendulumWithBotVis : public Pendulum {
public:
  PendulumWithBotVis(const std::shared_ptr<lcm::LCM>& lcm) : Pendulum(lcm), botvis(lcm,"Pendulum.urdf",DrakeJoint::FIXED) {}

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    botvis.output(t,Eigen::VectorXd::Zero(0),x);
    return Pendulum::output(t,x,u);
  }

  BotVisualizer botvis;
};

class PendulumEnergyShaping : public DrakeSystem {
public:
  PendulumEnergyShaping(const Pendulum& pendulum)
          : DrakeSystem("PendulumEnergyShaping"),
            m(pendulum.m),
            l(pendulum.l),
            b(pendulum.b),
            g(pendulum.g)
  {
    input_frame = pendulum.output_frame;
    output_frame = pendulum.input_frame;
  };

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& unused, const Eigen::VectorXd& u) const override {
    double Etilde = .5 * m*l*l*u(1)*u(1) - m*g*l*cos(u(0)) - 1.1*m*g*l;
    Eigen::VectorXd y(1);
    y << b*u(1) - .1*u(1)*Etilde;
    return y;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return true; }

  double m,l,b,g;  // pendulum parameters (initialized in the constructor)
};

#endif // _PENDULUM_H_

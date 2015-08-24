#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <iostream>
#include "DrakeSystem.h"


class Pendulum : public DrakeSystem {
public:
  Pendulum(void) :
          DrakeSystem("Pendulum",
                  std::make_shared<CoordinateFrame>("PendulumContState",std::vector<std::string>({"theta","thetadot"})),
                  nullptr,
                  std::make_shared<CoordinateFrame>("PendulumInput",std::vector<std::string>({"tau"})),
                  nullptr),
          m(1.0), // kg
          l(.5),  // m
          b(0.1), // kg m^2 /s
          lc(.5), // m
          I(.25), // m*l^2; % kg*m^2
          g(9.81) // m/s^2
  {
    output_frame = continuous_state_frame;
  }
  virtual ~Pendulum(void) {};

  virtual VectorXs dynamics(double t, const VectorXs& x, const VectorXs& u) const override {
    VectorXs xdot(2);
    xdot(0) = x(1);
    xdot(1) = (u(0) - m*g*lc*sin(x(0)) - b*x(1))/I;
    return xdot;
  }

  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u) const override {
    VectorXs y=x;
    return y;
  }

  virtual bool isTimeInvariant(void) { return true; }
  virtual bool isDirectFeedthrough(void) { return false; }

  double m,l,b,lc,I,g;  // pendulum parameters (initialized in the constructor)
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

  virtual VectorXs output(double t, const VectorXs& unused, const VectorXs& u) const override {
    double Etilde = .5 * m*l*l*u(1)*u(1) - m*g*l*cos(u(0)) - 1.1*m*g*l;
    VectorXs y(1);
    y << b*u(1) - .1*u(1)*Etilde;
    return y;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return true; }

  double m,l,b,g;  // pendulum parameters (initialized in the constructor)
};

#endif // _PENDULUM_H_

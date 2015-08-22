#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <iostream>
#include "DrakeSystem.h"

using namespace std;
using namespace Eigen;


class Pendulum : public DrakeSystem {
public:
  Pendulum(void) :
          DrakeSystem("Pendulum",2,0,1,2),
          m(1.0), // kg
          l(.5),  // m
          b(0.1), // kg m^2 /s
          lc(.5), // m
          I(.25), // m*l^2; % kg*m^2
          g(9.81) // m/s^2
  {
    input_frame->setCoordinateNames({"tau"});
    continuous_state_frame->setCoordinateNames({"theta","thetadot"});
    output_frame = continuous_state_frame;
  }
  virtual ~Pendulum(void) {};

  virtual VectorXs dynamics(double t, const VectorXs& x, const VectorXs& u) {
    VectorXd xdot(2);
    xdot(0) = x(1);
    xdot(1) = (u(0) - m*g*lc*sin(x(0)) - b*x(1))/I;
    return xdot;
  }

  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u) {
    VectorXd y=x;
    return y;
  }

  virtual bool isTimeInvariant(void) { return true; }
  virtual bool isDirectFeedthrough(void) { return false; }

  double m,l,b,lc,I,g;  // pendulum parameters (initialized in the constructor)
};

class PendulumEnergyShaping : public DrakeSystem {
public:
  PendulumEnergyShaping(const shared_ptr<Pendulum>& pendulum)
          : DrakeSystem("PendulumEnergyShaping"),
            m(pendulum->m),
            l(pendulum->l),
            b(pendulum->b),
            g(pendulum->g)
  {
    input_frame = pendulum->output_frame;
    output_frame = pendulum->input_frame;
  };

  virtual VectorXs output(double t, const VectorXs& unused, const VectorXs& x) {
    double Etilde = .5 * m*l*l*x(1)*x(1) - m*g*l*cos(x(0)) - 1.1*m*g*l;
    VectorXs u(1);
    u << b*x(1) - .1*x(1)*Etilde;
    return u;
  }

  virtual bool isTimeInvariant(void) { return true; }
  virtual bool isDirectFeedthrough(void) { return true; }

  double m,l,b,g;  // pendulum parameters (initialized in the constructor)
};

#endif // _PENDULUM_H_

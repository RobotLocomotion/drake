#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <iostream>
#include "DrakeSystem.h"

using namespace std;
using namespace Eigen;


class Pendulum : public DrakeSystem {
public:
  Pendulum(void) : DrakeSystem("Pendulum",2,0,1,1) {
    input_frame->setCoordinateNames({"tau"});
    continuous_state_frame->setCoordinateNames({"theta","thetadot"});
    output_frame->setCoordinateNames({"theta"});
  }
  virtual ~Pendulum(void) {};

  virtual VectorXs dynamics(double t, const VectorXs& x, const VectorXs& u) {
    VectorXd xdot(2);
    xdot(0) = x(1);
    xdot(1) = (u(0) - m*g*lc*sin(x(0)) - b*x(1))/I;
    return xdot;
  }

  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u) {
    VectorXd y(1);
    y(0) = x(0);
    cout << y << endl;
    return y;
  }

private:
  const double m = 1.0; // kg
  const double l = .5;  // m
  const double b = 0.1; // kg m^2 /s
  const double lc = .5; // m
  const double I = .25; // m*l^2; % kg*m^2
  const double g = 9.81; // m/s^2
};

#endif // _PENDULUM_H_

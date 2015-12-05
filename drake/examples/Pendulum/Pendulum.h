#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <iostream>
#include <cmath>
#include "DrakeSystem.h"
//#include "LCMCoordinateFrame.h"
//#include "BotVisualizer.h"

using namespace std;

template <typename ScalarType = double>
class PendulumState  { // models the Drake::Vector concept
public:
  PendulumState(void) : theta(0), thetadot(0) {};
  PendulumState(const Eigen::Matrix<ScalarType,2,1>& x) : theta(x(0)), thetadot(x(1)) {};

  PendulumState& operator=(const Eigen::Matrix<ScalarType,2,1>& x) {
    theta = x(0);
    thetadot = x(1);
    return *this;
  }

  operator Eigen::Matrix<ScalarType,2,1> () const {
    Eigen::Matrix<ScalarType,2,1> x;
    x << theta, thetadot;
    return x;
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumState& x)
  {
    os << "  theta = " << x.theta << endl;
    os << "  thetadot = " << x.thetadot << endl;
    return os;
  }

  static std::size_t size() { return 2; }

  ScalarType theta;
  ScalarType thetadot;
};


template <typename ScalarType = double>
class PendulumInput {
public:
  PendulumInput(void) : tau(0) {};
  PendulumInput(const Eigen::Matrix<ScalarType,1,1>& x) : tau(x(0)) {};

  PendulumInput& operator=(const Eigen::Matrix<ScalarType,1,1>& x) {
    tau = x(0);
    return *this;
  }

  operator Eigen::Matrix<ScalarType,1,1> () const {
    Eigen::Matrix<ScalarType,1,1> x;
    x << tau;
    return x;
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumInput& x)
  {
    os << "  tau = " << x.tau << endl;
    return os;
  }

  static unsigned int size() { return 1; }

  ScalarType tau;
};

namespace Drake {

  template <typename ScalarType>
  struct VectorTraits<PendulumState<ScalarType> > {
    enum {
      RowsAtCompileTime = 2
    };
  };

  template <typename ScalarType>
  struct VectorTraits<PendulumInput<ScalarType> > {
    enum {
      RowsAtCompileTime = 1
    };
  };

}

class Pendulum : public Drake::System<Pendulum,PendulumState,PendulumInput,PendulumState,false,false> {
public:
  Pendulum() :
          m(1.0), // kg
          l(.5),  // m
          b(0.1), // kg m^2 /s
          lc(.5), // m
          I(.25), // m*l^2; % kg*m^2
          g(9.81) // m/s^2
  {}
  virtual ~Pendulum(void) {};

/*
  DrakeSystemPtr balanceLQR() {
    Eigen::MatrixXd Q(2,2);  Q << 10, 0, 0, 1;
    Eigen::MatrixXd R(1,1);  R << 1;
    Eigen::VectorXd xG(2);   xG << M_PI, 0;
    Eigen::VectorXd uG(1);   uG << 0;

    return timeInvariantLQR(xG,uG,Q,R);
  }
*/

  template <typename ScalarType>
  PendulumState<ScalarType> dynamicsImplementation(const PendulumState<ScalarType>& x, const PendulumInput<ScalarType>& u) const {
    PendulumState<ScalarType> dot;
    dot.theta = x.thetadot;
    dot.thetadot = (u.tau - m*g*lc*sin(x.theta) - b*x.thetadot)/I;
    return dot;
  }


  template <typename ScalarType>
  PendulumState<ScalarType> outputImplementation(const PendulumState<ScalarType>& x) const {
    return x;
  }

public:
  double m,l,b,lc,I,g;  // pendulum parameters (initialized in the constructor)
};

/*
class PendulumWithBotVis : public Pendulum {
public:
  PendulumWithBotVis(const std::shared_ptr<lcm::LCM>& lcm) : Pendulum(lcm), botvis(lcm,"Pendulum.urdf",DrakeJoint::FIXED) {}

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    botvis.output(t,Eigen::VectorXd::Zero(0),x);
    return Pendulum::output(t,x,u);
  }

  BotVisualizer botvis;
};
*/

class PendulumEnergyShaping : public Drake::System<PendulumEnergyShaping,Drake::UnusedVector,PendulumState,PendulumInput,false,true> {
public:
  PendulumEnergyShaping(const Pendulum& pendulum)
          : m(pendulum.m),
            l(pendulum.l),
            b(pendulum.b),
            g(pendulum.g)
  {};

  template <typename ScalarType>
  PendulumInput<ScalarType> outputImplementation(const Drake::UnusedVector<ScalarType>& unused, const PendulumState<ScalarType>& x) const {
    ScalarType Etilde = .5 * m*l*l*x.thetadot*x.thetadot - m*g*l*cos(x.theta) - 1.1*m*g*l;
    PendulumInput<ScalarType> u;
    u.tau = b*x.thetadot - .1*x.thetadot*Etilde;
    return u;
  }

  double m,l,b,g;  // pendulum parameters (initialized in the constructor)
};

#endif // _PENDULUM_H_

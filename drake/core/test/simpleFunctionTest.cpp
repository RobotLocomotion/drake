
#include "../DrakeCore.h"
#include <iostream>

using namespace Drake;
using namespace std;

template <typename ScalarType = double>
class PendulumState : public Vector<ScalarType,2> {
public:
  virtual Eigen::Matrix<ScalarType,2,1> eigen() const {
    Eigen::Matrix<ScalarType,2,1> x;
    x(0) = theta;
    x(1) = thetadot;
    return x;
  }

  virtual std::string getCoordinateName(unsigned int i) const {
    switch(i) {
      case 0: return "theta";
      case 1: return "thetadot";
      default: throw std::runtime_error("bad coordinate number");
    }
  };

  ScalarType theta;
  ScalarType thetadot;
};

template <typename ScalarType = double>
class PendulumInput : public Vector<ScalarType,1> {
public:
  virtual Eigen::Matrix<ScalarType,1,1> eigen() const {
    Eigen::Matrix<ScalarType,1,1> u;
    u(0) = tau;
    return u;
  }

  virtual std::string getCoordinateName(unsigned int i) const {
    switch(i) {
      case 0: return "tau";
      default: throw std::runtime_error("bad coordinate number");
    }
  };

  ScalarType tau;
};

template <typename ScalarType, template<typename> class StateVector, template<typename> class InputVector>
class SystemVector : Vector<ScalarType,-1> // todo: get size from state and input vector template args
{
public:
  virtual Eigen::Matrix<ScalarType,-1,1> eigen() const override {
    Eigen::Matrix<ScalarType,-1,1> vec(1+x.size()+u.size());
    vec(0) = t;
    vec.middleRows(1,x.size()) = x.eigen();
    vec.bottomRows(u.size()) = u.eigen();
    return vec;
  }

  virtual unsigned int size() const override {
    return 1+x.size()+u.size();
  }
  virtual std::string getCoordinateName(unsigned int i) const override {
    if (i==0) { return "t"; }
    else if (i<1+x.size()) { return x.getCoordinateName(i-1); }
    else { return u.getCoordinateName(i-1-x.size()); }
  };

  ScalarType t;
  StateVector<ScalarType> x;
  InputVector<ScalarType> u;
};


class PendulumPassiveDynamics : public Function<PendulumState,PendulumState> {
public:
  template <typename ScalarType>
  PendulumState<ScalarType> eval(const PendulumState<ScalarType>& x) const {
    PendulumState<ScalarType> xdot;
    xdot.theta = x.thetadot;
    xdot.thetadot = sin(x.theta);
    return xdot;
  }
};



int main(int argc, char* argv[])
{
  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = .3;
  cout << "x = " << endl << state << endl;

  PendulumInput<double> input;
  input.tau = 0.0;

  PendulumPassiveDynamics passiveDynamics;
  cout << "xdot = " << endl << passiveDynamics.eval(state) << endl;

  SystemVector<double,PendulumState,PendulumInput> txu;
  txu.t = (double) 0.0;
  txu.x = state;
  txu.u = input;

  return 0;
}
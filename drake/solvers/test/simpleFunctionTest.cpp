
#include "Core.h"
#include "DrakeFunction.h"
#include <iostream>

using namespace Drake;
using namespace std;

template <typename ScalarType = double>
class PendulumState  { // models the Drake::Vector concept
public:
  PendulumState(void) {
    theta = 0; thetadot = 0;
  }
  PendulumState(const Eigen::Matrix<ScalarType,2,1>& x) {
    theta = x(0);
    thetadot = x(1);
  }

  operator Eigen::Matrix<ScalarType,2,1> () const {
    Eigen::Matrix<ScalarType,2,1> x;
    x(0) = theta;
    x(1) = thetadot;
    return x;
  }

  operator Eigen::Matrix<ScalarType,-1,1> () const {
    return Eigen::Matrix<ScalarType,2,1>();
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumState& x)
  {
    os << "  theta = " << x.theta << endl;
    os << "  thetadot = " << x.thetadot << endl;
    return os;
  }

  ScalarType theta;
  ScalarType thetadot;
};

template <typename ScalarType = double>
class PendulumInput {
public:
  virtual Eigen::Matrix<ScalarType,1,1> eigen() const {
    Eigen::Matrix<ScalarType,1,1> u;
    u(0) = tau;
    return u;
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumInput& x)
  {
    os << "  tau = " << x.tau << endl;
    return os;
  }

  ScalarType tau;
};

/*
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
 */


class PendulumPassiveDynamics : public Function<PendulumPassiveDynamics,PendulumState,PendulumState> {
public:
  template <typename ScalarType>
  PendulumState<ScalarType> eval(const PendulumState<ScalarType>& x) const {
    PendulumState<ScalarType> xdot;
    xdot.theta = x.thetadot;
    xdot.thetadot = sin(x.theta);
    return xdot;
  }
};

// note: uses CRTP
template <typename Derived, template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector >
class System {
public:
  virtual StateVector<double> dynamics(const double t, const StateVector<double>& x, const InputVector<double>& u)  { // todo: add const (w/o compile error)?
    return static_cast<Derived*>(this)->evalDynamics(t,x,u);
  };

  virtual OutputVector<double> output(const double t, const StateVector<double>& x, const InputVector<double>& u)  { // todo: add const (w/o compile error)?
    return static_cast<Derived*>(this)->evalOutput(t,x,u);
  };

  // derived classes must implement, e.g.
  // template <typename ScalarType>
  // OutputVector<ScalarType> eval(const ScalarType t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const;

  // sparsity
};

/*
class PendulumDynamics : public SystemFunction<PendulumDynamics,PendulumState,PendulumInput,PendulumState> {
public:
  template <typename ScalarType>
  PendulumState<ScalarType> eval(const ScalarType t, const PendulumState<ScalarType>& x, const PendulumInput<ScalarType>& u) const {
    PendulumState<ScalarType> xdot;
    xdot.theta = x.thetadot;
    xdot.thetadot = u.tau+sin(x.theta);
    return xdot;
  }
};
*/

int main(int argc, char* argv[])
{
  Eigen::Vector2d x;  x << 0.2, 0.4;

  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = .3;
  cout << "state = " << endl << state << endl;

  state = x;
  cout << "state = " << endl << state << endl;

  state.theta = 0.4;
  x = state;
  cout << "x = " << x.transpose() << endl << endl;

  {
    Eigen::Vector2d y = state;
    cout << "y = " << y.transpose() << endl << endl;
  }
  {
    Eigen::VectorXd y = state;
    cout << "y = " << y.transpose() << endl << endl;
  }

  PendulumInput<double> input;
  input.tau = 0.2;

  PendulumPassiveDynamics passiveDynamics;
  cout << "xdot (passive) = " << endl << passiveDynamics(state) << endl;

//  PendulumDynamics dynamics;
//  cout << "xdot = " << endl << dynamics(0.0,state,input) << endl;

  return 0;
}
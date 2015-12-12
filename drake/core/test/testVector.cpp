
#include "Pendulum.h"  // to get some types
#include <iostream>

using namespace std;
using namespace Drake;

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
  Eigen::VectorXd y = static_cast<Eigen::Vector2d>(state);
  cout << "y = " << y.transpose() << endl << endl;
  }

  PendulumInput<double> input;
  input.tau = 0.2;

  Eigen::Vector3d abc;  abc << 1,2,3;
  {
    Drake::CombinedVector<double, PendulumState, PendulumInput> test(abc);
    cout << test << endl;
    test=2*abc;
    cout << test << endl;
    cout << "type: " << typeid(test).name() << endl;
  }
  {
    Drake::CombinedVectorBuilder<PendulumState,PendulumInput>::VecType<double> test(abc);
    cout << test << endl;
    test=2*abc;
    cout << test << endl;
  }
  {
    // combining a vector with an unused or empty vector should return the original type
    PendulumState<double> ps;
    {
      Drake::CombinedVectorBuilder<PendulumState, UnusedVector>::VecType<double> test;
      assert(typeid(ps).hash_code() == typeid(test).hash_code());
    }
    {
      Drake::CombinedVectorBuilder<UnusedVector, PendulumState>::VecType<double> test;
      assert(typeid(ps).hash_code() == typeid(test).hash_code());
    }
    {
      Drake::CombinedVectorBuilder<EmptyVector, PendulumState>::VecType<double> test;
      assert(typeid(ps).hash_code() == typeid(test).hash_code());
    }
  }

  return 0;
}


#include "Pendulum.h"  // to get some types
#include <iostream>

using namespace std;
using namespace Drake;

struct OutputTest {
    template <typename ScalarType> using OutputVector = EigenVector<2>::type<ScalarType>;

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t) const;
};

struct OutputTestTwo {
    template <typename ScalarType> using OutputVector = EigenVector<2>::type<ScalarType>;

    OutputVector<double> output(const double& t) const;
};

int main(int argc, char* argv[])
{
  Eigen::Vector2d x;  x << 0.2, 0.4;

  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = .3;

  assert(size(state)==2);

  state = x;
  assert(state.thetadot == 0.4);

  state.theta = 0.5;
  x = toEigen(state);
  assert(x(0) = 0.5);

  {
    Eigen::VectorXd y = toEigen(state);
    assert((x - y).isZero());
  }

  PendulumInput<double> input;
  input.tau = 0.2;

  Eigen::Vector3d abc;  abc << 1,2,3;
  {
    Drake::CombinedVector<double, PendulumState, PendulumInput> test(abc);
    test=2*abc;
    assert(test.first().theta == 2);
    assert(test.first().thetadot == 4);
    assert(test.second().tau == 6);
  }
  {
    Drake::CombinedVectorBuilder<PendulumState,PendulumInput>::VecType<double> test(abc);
    test=2*abc;
    assert(test.first().theta == 2);
    assert(test.first().thetadot == 4);
    assert(test.second().tau == 6);
  }
  {
    // combining a vector with an unused or empty vector should return the original type
    PendulumState<double> ps;
    {
      Drake::CombinedVectorBuilder<PendulumState, NullVector>::VecType<double> test;
      assert(typeid(ps).hash_code() == typeid(test).hash_code());
    }
    {
      Drake::CombinedVectorBuilder<NullVector, PendulumState>::VecType<double> test;
      assert(typeid(ps).hash_code() == typeid(test).hash_code());
    }
  }

  static_assert(Eigen::Matrix<double,2,1>::RowsAtCompileTime == 2,"failed to evaluate RowsAtCompileTime");

/*
  { // test for a polynomial-based algorithm
    static_assert(isPolynomial<Pendulum>,"requires polynomial dynamics");

    PendulumState<Polynomial<double>> x;
    PendulumInput<Polynomial<double>> u;
    auto out = p->dynamicsImplementation(x,u);
  }
*/



  return 0;
}

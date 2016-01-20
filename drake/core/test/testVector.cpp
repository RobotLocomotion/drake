
#include "drake/examples/Pendulum/Pendulum.h"  // to get some types
#include "drake/util/testUtil.h"

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
  try {
  Eigen::Vector2d x;  x << 0.2, 0.4;

  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = .3;

  valuecheck(size(state),static_cast<size_t>(2));

  state = x;
  valuecheck(state.thetadot,0.4);

  state.theta = 0.5;
  x = toEigen(state);
  valuecheck(x(0),0.5);

  {
    Eigen::VectorXd y = toEigen(state);
    valuecheckMatrix(x,y,1e-8);
  }

  PendulumInput<double> input;
  input.tau = 0.2;

  Eigen::Vector3d abc;  abc << 1,2,3;
  {
    CombinedVector<double, PendulumState, PendulumInput> test(abc);
    test=2*abc;
    valuecheck(test.first().theta,2.0);
    valuecheck(test.first().thetadot,4.0);
    valuecheck(test.second().tau,6.0);
  }
  {
    CombinedVectorUtil<PendulumState,PendulumInput>::type<double> test(abc);
    test=2*abc;
    valuecheck(test.first().theta,2.0);
    valuecheck(test.first().thetadot,4.0);
    valuecheck(test.second().tau,6.0);
  }
  {
    // combining a vector with an unused or empty vector should return the original type
    {
      CombinedVectorUtil<PendulumState, NullVector>::type<double> test;
      if (!is_same<PendulumState<double>,decltype(test)>::value)
	throw std::runtime_error("combined vector builder returned " + static_cast<string>(typeid(test).name()));
    }
    {
      CombinedVectorUtil<NullVector, PendulumState>::type<double> test;
      if (!is_same<PendulumState<double>,decltype(test)>::value)
	throw std::runtime_error("combined vector builder returned " + static_cast<string>(typeid(test).name()));
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
    if (!InputOutputRelation::isa(InputOutputRelation::LINEAR,InputOutputRelation::POLYNOMIAL))
      throw runtime_error("linear is polynomial");

    if (!InputOutputRelation::isa(InputOutputRelation::ZERO,InputOutputRelation::ARBITRARY))
      throw runtime_error("linear is arbitrary");

    if (InputOutputRelation::leastCommonAncestor({InputOutputRelation::AFFINE,InputOutputRelation::LINEAR,InputOutputRelation::POLYNOMIAL})!=InputOutputRelation::POLYNOMIAL)
      throw runtime_error("lca should be poly");

    {
      InputOutputRelation g(InputOutputRelation::LINEAR);
      InputOutputRelation f(InputOutputRelation::POLYNOMIAL);
      if (InputOutputRelation::composeWith(g,f).form != InputOutputRelation::POLYNOMIAL)
        throw runtime_error("should be poly");
      if (InputOutputRelation::composeWith(f,g).form != InputOutputRelation::POLYNOMIAL)
        throw runtime_error("should be poly");
      if (InputOutputRelation::combine(g,f).form != InputOutputRelation::POLYNOMIAL)
        throw runtime_error("should be poly");
    }

  } catch (const exception& e) {
    cout << "ERROR: " << e.what() << endl;
    return -1;
  }


  cout << "all tests passed" << endl;
  return 0;
}

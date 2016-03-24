#include "drake/examples/Pendulum/Pendulum.h"  // to get some types
#include "drake/util/testUtil.h"

#include "gtest/gtest.h"
#include "drake/core/test/eigen_compare_matrix.h"

namespace Drake {
namespace {

struct OutputTest {
  template <typename ScalarType>
  using OutputVector = EigenVector<2>::type<ScalarType>;

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t) const;
};

struct OutputTestTwo {
  template <typename ScalarType>
  using OutputVector = EigenVector<2>::type<ScalarType>;

  OutputVector<double> output(const double& t) const;
};

// Tests the ability to set a PendulumState equal to a vector and vice versa.
TEST(VectorTest, ValueAssignment) {
  Eigen::Vector2d x;
  x << 0.2, 0.4;

  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = .3;

  EXPECT_EQ(size(state), static_cast<size_t>(2));

  state = x;
  EXPECT_EQ(state.thetadot, 0.4);

  state.theta = 0.5;
  x = toEigen(state);
  EXPECT_EQ(x(0), 0.5);

  {
    Eigen::VectorXd y = toEigen(state);
    EXPECT_THAT(x, EigenMatrixIsApproximatelyEqual(y));

    // Eigen::Vector2d yy;
    // yy << 0.5, 0.6;
    // EXPECT_THAT(x, EigenMatrixIsApproximatelyEqual(yy));

    // valuecheckMatrix(x, y, 1e-8);         // TODO: Replace with Google Mock!

    // EXPECT_THAT(4.0, EigenMatrixIsApproximatelyEqual(4.0));
  }
}

// Test the CombinedVector
TEST(VectorTest, CombinedVector) {

  Eigen::Vector3d abc;
  abc << 1, 2, 3;
  {
    CombinedVector<double, PendulumState, PendulumInput> test(abc);
    test = 2 * abc;
    EXPECT_EQ(test.first().theta, 2.0);
    EXPECT_EQ(test.first().thetadot, 4.0);
    EXPECT_EQ(test.second().tau, 6.0);
  }
  {
    CombinedVectorUtil<PendulumState, PendulumInput>::type<double> test(abc);
    test = 2 * abc;
    EXPECT_EQ(test.first().theta, 2.0);
    EXPECT_EQ(test.first().thetadot, 4.0);
    EXPECT_EQ(test.second().tau, 6.0);
  }

  // combining a vector with an unused or empty vector should return the
  // original type
  {
    CombinedVectorUtil<PendulumState, NullVector>::type<double> test;
    EXPECT_TRUE((is_same<PendulumState<double>, decltype(test)>::value))
      << "combined vector builder returned " +
         static_cast<string>(typeid(test).name());
  }
  {
    CombinedVectorUtil<NullVector, PendulumState>::type<double> test;
    EXPECT_TRUE((is_same<PendulumState<double>, decltype(test)>::value))
      << "combined vector builder returned " +
         static_cast<string>(typeid(test).name());
  }
}

// Test the RowsAtCompileTime
TEST(VectorTest, RowsAtCompileTime) {
  EXPECT_TRUE((Eigen::Matrix<double, 2, 1>::RowsAtCompileTime == 2))
    << "failed to evaluate RowsAtCompileTime";
}

// Test the isPolynomial
// TEST(VectorTest, PolynomialBasedAlgorithm) {
//   // test for a polynomial-based algorithm
//   static_assert(isPolynomial<Pendulum>,"requires polynomial dynamics");

//   PendulumState<Polynomial<double>> x;
//   PendulumInput<Polynomial<double>> u;
//   auto out = p->dynamicsImplementation(x, u);
// }

// Test the InputOutputRelation
TEST(VectorTest, InputOutputRelation) {
  EXPECT_TRUE((InputOutputRelation::isA(InputOutputRelation::Form::LINEAR,
                                InputOutputRelation::Form::POLYNOMIAL)))
    << "linear is polynomial";

  EXPECT_TRUE((InputOutputRelation::isA(InputOutputRelation::Form::ZERO,
                                InputOutputRelation::Form::ARBITRARY)))
    << "linear is arbitrary";

  EXPECT_TRUE((InputOutputRelation::leastCommonAncestor(
          {InputOutputRelation::Form::AFFINE,
           InputOutputRelation::Form::LINEAR,
           InputOutputRelation::Form::POLYNOMIAL}) ==
      InputOutputRelation::Form::POLYNOMIAL))
  << "lca should be poly";

  {
    InputOutputRelation g(InputOutputRelation::Form::LINEAR);
    InputOutputRelation f(InputOutputRelation::Form::POLYNOMIAL);

    EXPECT_TRUE(InputOutputRelation::composeWith(g, f).form ==
        InputOutputRelation::Form::POLYNOMIAL)
      << "should be poly";

    EXPECT_TRUE(InputOutputRelation::composeWith(f, g).form ==
        InputOutputRelation::Form::POLYNOMIAL)
      << "should be poly";

    EXPECT_TRUE(InputOutputRelation::combine(g, f).form ==
        InputOutputRelation::Form::POLYNOMIAL)
      << "should be poly";
  }

}

}  // namespace
}  // namespace Drake

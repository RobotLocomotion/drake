#include "drake/common/symbolic_decompose.h"

#include <stdexcept>

#include <gtest/gtest.h>

namespace drake {
namespace symbolic {
namespace {

using std::runtime_error;

class SymbolicDecomposeTest : public ::testing::Test {
 public:
  SymbolicDecomposeTest() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SymbolicDecomposeTest)

 protected:
  void SetUp() override {
    // clang-format off
  M_  << 1, 0, 3,
        -4, 5, 0,
         7, 0, 9;
  v_ << 3,
       -7,
        2;
  x_ << x0_, x1_, x2_;
    // clang-format on
  }

  const Variable x0_{"x0"};
  const Variable x1_{"x1"};
  const Variable x2_{"x2"};
  VectorX<Variable> x_{3};
  const Variable a_{"a"};
  const Variable b_{"b"};
  const Variable c_{"c"};

  // M and v matrices that we pass to decompose functions as input. We mix
  // static-sized and dynamic-sized matrices intentionally.
  Eigen::Matrix3d M_;
  Eigen::Vector3d v_;

  // Expected values for M and v.
  Eigen::Matrix3d M_expected_static_;
  Eigen::MatrixXd M_expected_dynamic_{3, 3};
  Eigen::Vector3d v_expected_static_;
  Eigen::VectorXd v_expected_dynamic_{3};

  // Extra terms that we add to test exceptional cases.
  Eigen::Matrix<Expression, 3, 1> extra_terms_;
};

TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsDynamic) {
  DecomposeLinearExpressions(M_ * x_, x_, &M_expected_dynamic_);
  EXPECT_EQ(M_expected_dynamic_, M_);
}

TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsStatic) {
  DecomposeLinearExpressions(M_ * x_, x_, &M_expected_static_);
  EXPECT_EQ(M_expected_static_, M_);
}

// Adds quadratic terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsExceptionNonLinear) {
  // clang-format off
  extra_terms_ << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeLinearExpressionsExceptionNonPolynomial) {
  // clang-format off
  extra_terms_ << sin(x0_),
                 cos(x1_),
                 log(x2_);
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

// Adds terms with non-const coefficients to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeLinearExpressionsExceptionNonConstCoefficient) {
  // clang-format off
  extra_terms_ << a_ * x0_,
                  b_ * x1_,
                  c_ * x2_;
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

// Adds constant terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsExceptionAffine) {
  // clang-format off
  extra_terms_ << -1,
                  1,
                  M_PI;
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsDynamic) {
  DecomposeAffineExpressions(M_ * x_ + v_, x_, &M_expected_dynamic_,
                             &v_expected_dynamic_);
  EXPECT_EQ(M_expected_dynamic_, M_);
  EXPECT_EQ(v_expected_dynamic_, v_);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsStatic) {
  DecomposeAffineExpressions(M_ * x_ + v_, x_, &M_expected_static_,
                             &v_expected_static_);
  EXPECT_EQ(M_expected_static_, M_);
  EXPECT_EQ(v_expected_static_, v_);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsBlock) {
  Eigen::Matrix4d M_expected;
  auto block = M_expected.block(0, 0, 3, 3);
  DecomposeAffineExpressions(M_ * x_ + v_, x_, &block, &v_expected_static_);
  EXPECT_EQ(M_expected.block(0, 0, 3, 3), M_);
  EXPECT_EQ(v_expected_static_, v_);
}

// Adds quadratic terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsExceptionNonlinear) {
  // clang-format off
  extra_terms_ << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M_ * x_ + v_ + extra_terms_, x_,
                                 &M_expected_static_, &v_expected_static_),
      runtime_error);
}

// Adds terms with non-const coefficients to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeAffineExpressionsExceptionNonConstCoefficient) {
  // clang-format off
  extra_terms_ << a_ * x0_,
                  b_ * x1_,
                  c_ * x2_;
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M_ * x_ + v_ + extra_terms_, x_,
                                 &M_expected_static_, &v_expected_static_),
      runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeAffineExpressionsExceptionNonPolynomial) {
  // clang-format off
  extra_terms_ << sin(x0_),
                 cos(x1_),
                 log(x2_);
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M_ * x_ + v_ + extra_terms_, x_,
                                 &M_expected_static_, &v_expected_static_),
      runtime_error);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

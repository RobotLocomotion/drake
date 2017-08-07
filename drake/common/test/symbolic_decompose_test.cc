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
  M1_  << 1, 0, 3,
        -4, 5, 0,
         7, 0, 9;
  M2_  << 10, -7,
         12, 0,
         0,  15;
  v_ << 3,
       -7,
        2;
  x_ << x0_, x1_, x2_;
  u_ << u0_, u1_;
    // clang-format on
  }

  const Variable x0_{"x0"};
  const Variable x1_{"x1"};
  const Variable x2_{"x2"};
  const Variable u0_{"u0"};
  const Variable u1_{"u1"};
  VectorX<Variable> x_{3};
  VectorX<Variable> u_{2};

  // M1, M2, and v matrices that we pass to decompose functions as input. We mix
  // static-sized and dynamic-sized matrices intentionally.
  Eigen::Matrix3d M1_;
  Eigen::MatrixXd M2_{3, 2};
  Eigen::Vector3d v_;

  // Expected values for M1, M2, and v.
  Eigen::Matrix3d M1_expected_static_;
  Eigen::MatrixXd M1_expected_dynamic_{3, 3};
  Eigen::Matrix<double, 3, 2> M2_expected_static_;
  Eigen::MatrixXd M2_expected_dynamic_{3, 2};
  Eigen::Vector3d v_expected_static_;
  Eigen::VectorXd v_expected_dynamic_{3};

  // Extra terms that we add to test exceptional cases.
  Eigen::Matrix<Expression, 3, 1> extra_terms_;
};

TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsDynamic) {
  DecomposeLinearExpressions(M1_ * x_ + M2_ * u_, x_, u_, M1_expected_dynamic_,
                             M2_expected_dynamic_);
  EXPECT_EQ(M1_expected_dynamic_, M1_);
  EXPECT_EQ(M2_expected_dynamic_, M2_);
}

TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsStatic) {
  DecomposeLinearExpressions(M1_ * x_ + M2_ * u_, x_, u_, M1_expected_static_,
                             M2_expected_static_);
  EXPECT_EQ(M1_expected_static_, M1_);
  EXPECT_EQ(M2_expected_static_, M2_);
}

// Adds quadratic terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsException1) {
  // clang-format off
  extra_terms_ << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(
      DecomposeLinearExpressions(M1_ * x_ + M2_ * u_ + extra_terms_, x_, u_,
                                 M1_expected_static_, M2_expected_static_),
      runtime_error);
}

// Adds bilinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsException2) {
  // clang-format off
  extra_terms_ << x0_ * u0_,
                 x1_ * u1_,
                 x2_ * u0_;
  // clang-format on
  EXPECT_THROW(
      DecomposeLinearExpressions(M1_ * x_ + M2_ * u_ + extra_terms_, x_, u_,
                                 M1_expected_static_, M2_expected_static_),
      runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsException3) {
  // clang-format off
  extra_terms_ << sin(x0_),
                 cos(x1_),
                 log(u0_);
  // clang-format on
  EXPECT_THROW(
      DecomposeLinearExpressions(M1_ * x_ + M2_ * u_ + extra_terms_, x_, u_,
                                 M1_expected_static_, M2_expected_static_),
      runtime_error);
}

// Adds constant terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsException4) {
  // clang-format off
  extra_terms_ << -1,
                  1,
                  M_PI;
  // clang-format on
  EXPECT_THROW(
      DecomposeLinearExpressions(M1_ * x_ + M2_ * u_ + extra_terms_, x_, u_,
                                 M1_expected_static_, M2_expected_static_),
      runtime_error);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsDynamic) {
  DecomposeAffineExpressions(M1_ * x_ + M2_ * u_ + v_, x_, u_,
                             M1_expected_dynamic_, M2_expected_dynamic_,
                             v_expected_dynamic_);
  EXPECT_EQ(M1_expected_dynamic_, M1_);
  EXPECT_EQ(M2_expected_dynamic_, M2_);
  EXPECT_EQ(v_expected_dynamic_, v_);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsStatic) {
  DecomposeAffineExpressions(M1_ * x_ + M2_ * u_ + v_, x_, u_,
                             M1_expected_static_, M2_expected_static_,
                             v_expected_static_);
  EXPECT_EQ(M1_expected_static_, M1_);
  EXPECT_EQ(M2_expected_static_, M2_);
  EXPECT_EQ(v_expected_static_, v_);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsBlock) {
  Eigen::Matrix4d M1_expected;
  Eigen::Matrix4d M2_expected;
  DecomposeAffineExpressions(M1_ * x_ + M2_ * u_ + v_, x_, u_,
                             M1_expected.block(0, 0, 3, 3),
                             M2_expected.block(1, 1, 3, 2), v_expected_static_);
  EXPECT_EQ(M1_expected.block(0, 0, 3, 3), M1_);
  EXPECT_EQ(M2_expected.block(1, 1, 3, 2), M2_);
  EXPECT_EQ(v_expected_static_, v_);
}

// Adds quadratic terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsException1) {
  // clang-format off
  extra_terms_ << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M1_ * x_ + M2_ * u_ + v_ + extra_terms_, x_,
                                 u_, M1_expected_static_, M2_expected_static_,
                                 v_expected_static_),
      runtime_error);
}

// Adds bilinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsException2) {
  // clang-format off
  extra_terms_ << x0_ * u0_,
                 x1_ * u1_,
                 x2_ * u0_;
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M1_ * x_ + M2_ * u_ + v_ + extra_terms_, x_,
                                 u_, M1_expected_static_, M2_expected_static_,
                                 v_expected_static_),
      runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsException3) {
  // clang-format off
  extra_terms_ << sin(x0_),
                 cos(x1_),
                 log(u0_);
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M1_ * x_ + M2_ * u_ + v_ + extra_terms_, x_,
                                 u_, M1_expected_static_, M2_expected_static_,
                                 v_expected_static_),
      runtime_error);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

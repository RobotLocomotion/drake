#include "drake/common/symbolic_expression.h"

#include <functional>

#include "gtest/gtest.h"

#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
namespace {

using std::ptr_fun;

class SymbolicExpressionMatrixTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const Expression zero_{0.0};
  const Expression one_{1.0};
  const Expression two_{2.0};
  const Expression neg_one_{-1.0};
  const Expression pi_{3.141592};
  const Expression neg_pi_{-3.141592};
  const Expression e_{2.718};

  Eigen::Matrix<Expression, 3, 2, Eigen::DontAlign> A_;
  Eigen::Matrix<Expression, 2, 3, Eigen::DontAlign> B_;
  Eigen::Matrix<Expression, 3, 2, Eigen::DontAlign> C_;

  void SetUp() override {
    // clang-format off
    A_ << x_, one_,       //  [x  1]
          y_, neg_one_,   //  [y -1]
          z_, pi_;        //  [z  3.141592]

    B_ << x_, y_,  z_,    //  [x     y        z]
          e_, pi_, two_;  //  [2.718 3.141592 2]

    C_ << z_, two_,       //  [z  2]
          x_, e_,         //  [x -2.718]
          y_, pi_;        //  [y  3.141592]
    // clang-format on
  }
};

TEST_F(SymbolicExpressionMatrixTest, EigenAdd) {
  auto const M(A_ + A_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ + x_), (one_ + one_),
                (y_ + y_), (neg_one_ + neg_one_),
                (z_ + z_), (pi_ + pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenSub1) {
  auto const M(A_ - A_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  EXPECT_EQ(M, M_expected);  // should be all zero.
}

TEST_F(SymbolicExpressionMatrixTest, EigenSub2) {
  auto const M(A_ - C_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ - z_), (one_ - two_),
                (y_ - x_), (neg_one_ - e_),
                (z_ - y_), (pi_ - pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);  // should be all zero.
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul1) {
  auto const M(A_ * B_);
  Eigen::Matrix<Expression, 3, 3> M_expected;
  // clang-format off
  M_expected <<
    (x_ * x_ + e_),       (x_ * y_ + pi_),       (x_ * z_ + two_),
    (y_ * x_ + -e_),      (y_ * y_ + - pi_),     (y_ * z_ + - two_),
    (z_ * x_ + pi_ * e_), (z_ * y_ + pi_ * pi_), (z_ * z_ + pi_ * two_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul2) {
  auto const M(B_ * A_);
  Eigen::Matrix<Expression, 2, 2> M_expected;
  // clang-format off
  M_expected <<
    (x_ * x_ + (y_ * y_ + z_ * z_)),    (x_ + (-y_ + z_ * pi_)),
    (e_ * x_ + (pi_ * y_ + two_ * z_)), (e_ * one_ + pi_ * - one_ + two_ * pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul3) {
  auto const M(2.0 * A_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (2 * x_), (2 * one_),
                (2 * y_), (2 * neg_one_),
                (2 * z_), (2 * pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul4) {
  auto const M(A_ * 2.0);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ * 2), (one_ * 2),
                (y_ * 2), (neg_one_ * 2),
                (z_ * 2), (pi_ * 2);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenDiv) {
  auto const M(A_ / 2.0);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ / 2), (one_ / 2),
                (y_ / 2), (neg_one_ / 2),
                (z_ / 2), (pi_ / 2);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, CheckStructuralEquality) {
  EXPECT_TRUE(CheckStructuralEquality(A_, A_));
  EXPECT_TRUE(CheckStructuralEquality(B_, B_));
  EXPECT_TRUE(CheckStructuralEquality(C_, C_));

  EXPECT_FALSE(CheckStructuralEquality(A_, C_));
  EXPECT_FALSE(CheckStructuralEquality(B_ * A_, B_ * C_));
}

// Checks if m1.array() == m2.array() returns an array whose (i, j) element is a
// formula m1(i, j) == m2(i, j).
bool CheckArrayOperatorEq(const MatrixX<Expression>& m1,
                          const MatrixX<Expression>& m2) {
  const auto arr = (m1.array() == m2.array());
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(m1(i, j) == m2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorEq) {
  const Eigen::Array<Formula, 3, 2> a1{A_.array() == A_.array()};
  const Eigen::Array<Formula, 2, 3> a2{B_.array() == B_.array()};
  const Eigen::Array<Formula, 3, 2> a3{C_.array() == C_.array()};
  EXPECT_TRUE(a1.unaryExpr(ptr_fun(is_true)).all());
  EXPECT_TRUE(a2.unaryExpr(ptr_fun(is_true)).all());
  EXPECT_TRUE(a3.unaryExpr(ptr_fun(is_true)).all());

  EXPECT_TRUE(CheckArrayOperatorEq(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorEq(B_ * A_, B_ * C_));
}

// Checks if m1 == m2 returns a formula which is a conjunction of
// m1(i, j) == m2(i, j) for all i and j.
bool CheckMatrixOperatorEq(const MatrixX<Expression>& m1,
                           const MatrixX<Expression>& m2) {
  const Formula f{m1 == m2};
  Formula f_expected{};  // True
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      f_expected = f_expected && (m1(i, j) == m2(i, j));
    }
  }
  return f.EqualTo(f_expected);
}

TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorEq1) {
  Eigen::Matrix<Expression, 2, 2> m1;
  Eigen::Matrix<Expression, 2, 2> m2;
  m1 << x_, y_, z_, x_;
  m2 << z_, x_, y_, z_;
  const Formula f{m1 == m2};
  EXPECT_EQ(f.to_string(), "((x = z) and (y = x) and (z = y))");
  ASSERT_TRUE(is_conjunction(f));
  EXPECT_EQ(get_operands(f).size(), 3);
  EXPECT_TRUE(CheckMatrixOperatorEq(m1, m2));
}

TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorEq2) {
  Eigen::Matrix<Expression, 2, 2> m1;
  Eigen::Matrix<Expression, 2, 2> m2;
  m1 << x_, 1.0, z_, x_;
  m2 << z_, 2.0, y_, z_;
  const Formula f1{m1 == m2};
  // Because (1.0 == 2.0) is false, the whole conjunction is reduced
  // to false.
  EXPECT_TRUE(is_false(f1));
  const Formula f2{m1 == m1};
  EXPECT_TRUE(is_true(f2));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

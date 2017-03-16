#include "drake/common/symbolic_expression.h"

#include <functional>

#include "gtest/gtest.h"

#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::equal_to;
using std::ptr_fun;
using test::FormulaEqual;

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

  Eigen::Matrix<Expression, 2, 2, Eigen::DontAlign> matrix_expr_1_;
  Eigen::Matrix<Expression, 2, 2, Eigen::DontAlign> matrix_expr_2_;
  Eigen::Matrix<Variable, 2, 2, Eigen::DontAlign> matrix_var_1_;
  Eigen::Matrix<Variable, 2, 2, Eigen::DontAlign> matrix_var_2_;
  Eigen::Matrix<double, 2, 2, Eigen::DontAlign> matrix_double_;

  Eigen::Array<Expression, 2, 2, Eigen::DontAlign> array_expr_1_;
  Eigen::Array<Expression, 2, 2, Eigen::DontAlign> array_expr_2_;
  Eigen::Array<Variable, 2, 2, Eigen::DontAlign> array_var_1_;
  Eigen::Array<Variable, 2, 2, Eigen::DontAlign> array_var_2_;
  Eigen::Array<double, 2, 2, Eigen::DontAlign> array_double_;

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

    matrix_expr_1_ << x_, y_,
                      z_, x_;
    matrix_expr_2_ << z_, x_,
                      y_, z_;
    matrix_var_1_ << var_x_, var_y_,
                     var_z_, var_x_;
    matrix_var_2_ << var_y_, var_z_,
                     var_x_, var_x_;
    matrix_double_ << 1.0, 2.0,
                      3.0, 4.0;
    // clang-format on

    array_expr_1_ = matrix_expr_1_.array();
    array_expr_2_ = matrix_expr_2_.array();
    array_var_1_ = matrix_var_1_.array();
    array_var_2_ = matrix_var_2_.array();
    array_double_ = matrix_double_.array();
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

// Given two Eigen arrays a1 and a2, it checks if a1 == a2 returns an array
// whose (i, j) element is a formula a1(i, j) == a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorEq(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 == a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) == a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() == m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) == m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorEq(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorEq(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 <= a2 returns an array
// whose (i, j) element is a formula a1(i, j) <= a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorLte(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 <= a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) <= a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() <= m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) <= m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorLte(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorLte(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 < a2 returns an array whose
// (i, j) element is a formula a1(i, j) < a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorLt(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 < a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) < a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() < m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) < m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorLt(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorLt(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 >= a2 returns an array
// whose (i, j) element is a formula a1(i, j) >= a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorGte(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 >= a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) >= a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() >= m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) >= m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorGte(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorGte(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 > a2 returns an array whose
// (i, j) element is a formula a1(i, j) > a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorGt(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 > a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) > a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() > m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) > m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorGt(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorGt(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 != a2 returns an array
// whose (i, j) element is a formula a1(i, j) != a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorNeq(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 != a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) != a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() != m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) != m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value,
    bool>::type
CheckArrayOperatorNeq(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorNeq(m1.array(), m2.array());
}

TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorExprEqExpr) {
  const Eigen::Array<Formula, 3, 2> a1{A_.array() == A_.array()};
  const Eigen::Array<Formula, 2, 3> a2{B_.array() == B_.array()};
  const Eigen::Array<Formula, 3, 2> a3{C_.array() == C_.array()};
  EXPECT_TRUE(a1.unaryExpr(ptr_fun(is_true)).all());
  EXPECT_TRUE(a2.unaryExpr(ptr_fun(is_true)).all());
  EXPECT_TRUE(a3.unaryExpr(ptr_fun(is_true)).all());
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Array<Expression>
TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorExprOpExpr) {
  EXPECT_TRUE(CheckArrayOperatorEq(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorEq(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorLte(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorLte(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorLt(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorLt(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorGte(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorGte(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorGt(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorGt(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorNeq(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorNeq(B_ * A_, B_ * C_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Array<Variable>
TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorExprOpVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_2_, array_expr_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Array<double>
TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorExprOpDouble) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_double_, array_expr_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and Array<double>
TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorVarOpDouble) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_double_, array_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and Array<Variable>
TEST_F(SymbolicExpressionMatrixTest, ArrayOperatorVarOpVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_2_, array_var_1_));
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

TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorExprEqExpr1) {
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

TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorExprEqExpr2) {
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

// Checks operator== between Matrix<Expression> and Matrix<Variable>
TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorExprEqVar) {
  Eigen::Matrix<Expression, 2, 2> m1;
  Eigen::Matrix<Variable, 2, 2> m2;
  m1 << x_, 1.0, z_, x_;
  m2 << var_x_, var_y_, var_z_, var_x_;
  const Formula f1{m1 == m2};
  const Formula f2{m2 == m1};
  EXPECT_PRED2(FormulaEqual, f1, 1.0 == var_y_);
  EXPECT_PRED2(FormulaEqual, f2, var_y_ == 1.0);
}

// Checks operator== between Matrix<Expression> and Matrix<double>
TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorExprEqDouble) {
  Eigen::Matrix<Expression, 2, 2> m1;
  Eigen::Matrix<double, 2, 2> m2;
  m1 << x_, 1.0, z_, x_;
  m2 << 1.0, 1.0, 3.0, 1.0;
  const Formula f1{m1 == m2};
  const Formula f2{m2 == m1};
  EXPECT_PRED2(FormulaEqual, f1, (x_ == 1.0) && (z_ == 3.0));
  EXPECT_PRED2(FormulaEqual, f2, (1.0 == x_) && (3.0 == z_));
}

// Checks operator== between Matrix<Variable> and Matrix<double>
TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorVarEqdouble) {
  Eigen::Matrix<Variable, 2, 2> m1;
  Eigen::Matrix<double, 2, 2> m2;
  m1 << var_x_, var_y_, var_z_, var_x_;
  m2 << 1.0, 2.0, 3.0, 1.0;
  const Formula f1{m1 == m2};
  const Formula f2{m2 == m1};
  EXPECT_PRED2(FormulaEqual, f1,
               (var_x_ == 1.0) && (var_y_ == 2.0) && (var_z_ == 3.0));
  EXPECT_PRED2(FormulaEqual, f2,
               (1.0 == var_x_) && (2.0 == var_y_) && (3.0 == var_z_));
}

// Checks operator== between Matrix<Variable> and Matrix<Variable>
TEST_F(SymbolicExpressionMatrixTest, MatrixOperatorVarEqVar) {
  Eigen::Matrix<Variable, 2, 2> m1;
  Eigen::Matrix<Variable, 2, 2> m2;
  m1 << var_x_, var_y_, var_z_, var_x_;
  m2 << var_y_, var_z_, var_x_, var_x_;
  const Formula f1{m1 == m2};
  const Formula f2{m2 == m1};
  EXPECT_PRED2(FormulaEqual, f1,
               (var_x_ == var_y_) && (var_y_ == var_z_) && (var_z_ == var_x_));
  EXPECT_PRED2(FormulaEqual, f2,
               (var_y_ == var_x_) && (var_z_ == var_y_) && (var_x_ == var_z_));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

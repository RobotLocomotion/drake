#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

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

TEST_F(SymbolicExpressionMatrixTest, GetVariableVector) {
  const Vector3<Expression> evec(x_, y_, z_);
  const VectorX<Variable> vec = GetVariableVector(evec);
  EXPECT_EQ(vec(0), x_);
  EXPECT_EQ(vec(1), y_);
  EXPECT_EQ(vec(2), z_);

  EXPECT_THROW(GetVariableVector(Vector3<Expression>(x_, y_, x_ + y_)),
               std::logic_error);
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

// Checks the following two formulas are identical:
//   - m1 == m2
//   - ⋀ᵢⱼ (m1.array() == m2.array())
bool CheckMatrixOperatorEq(const MatrixX<Expression>& m1,
                           const MatrixX<Expression>& m2) {
  const Formula f1{m1 == m2};
  const Formula f2{(m1.array() == m2.array()).redux(detail::logic_and)};
  return f1.EqualTo(f2);
}

// Checks the following two formulas are identical:
//   - m1 != m2
//   - ⋁ᵢⱼ (m1.array() != m2.array())
bool CheckMatrixOperatorNeq(const MatrixX<Expression>& m1,
                            const MatrixX<Expression>& m2) {
  const Formula f1{m1 != m2};
  const Formula f2{(m1.array() != m2.array()).redux(detail::logic_or)};
  return f1.EqualTo(f2);
}

// Checks the following two formulas are identical:
//   - m1 < m2
//   - ⋀ᵢⱼ (m1.array() < m2.array())
bool CheckMatrixOperatorLt(const MatrixX<Expression>& m1,
                           const MatrixX<Expression>& m2) {
  const Formula f1{m1 < m2};
  const Formula f2{(m1.array() < m2.array()).redux(detail::logic_and)};
  return f1.EqualTo(f2);
}

// Checks the following two formulas are identical:
//   - m1 <= m2
//   - ⋀ᵢⱼ (m1.array() <= m2.array())
bool CheckMatrixOperatorLte(const MatrixX<Expression>& m1,
                            const MatrixX<Expression>& m2) {
  const Formula f1{m1 <= m2};
  const Formula f2{(m1.array() <= m2.array()).redux(detail::logic_and)};
  return f1.EqualTo(f2);
}

// Checks the following two formulas are identical:
//   - m1 > m2
//   - ⋀ᵢⱼ (m1.array() > m2.array())
bool CheckMatrixOperatorGt(const MatrixX<Expression>& m1,
                           const MatrixX<Expression>& m2) {
  const Formula f1{m1 > m2};
  const Formula f2{(m1.array() > m2.array()).redux(detail::logic_and)};
  return f1.EqualTo(f2);
}

// Checks the following two formulas are identical:
//   - m1 >= m2
//   - ⋀ᵢⱼ (m1.array() >= m2.array())
bool CheckMatrixOperatorGte(const MatrixX<Expression>& m1,
                            const MatrixX<Expression>& m2) {
  const Formula f1{m1 >= m2};
  const Formula f2{(m1.array() >= m2.array()).redux(detail::logic_and)};
  return f1.EqualTo(f2);
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
  EXPECT_TRUE(CheckMatrixOperatorNeq(m1, m2));
  EXPECT_TRUE(CheckMatrixOperatorLt(m1, m2));
  EXPECT_TRUE(CheckMatrixOperatorLte(m1, m2));
  EXPECT_TRUE(CheckMatrixOperatorGt(m1, m2));
  EXPECT_TRUE(CheckMatrixOperatorGte(m1, m2));
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

TEST_F(SymbolicExpressionMatrixTest, ExpressionMatrixSegment) {
  Eigen::Matrix<Expression, 5, 1> v;
  v << x_, 1, y_, x_, 1;
  const auto s1 = v.segment(0, 2);  // [x, 1]
  const auto s2 = v.segment(1, 2);  // [1, y]
  const auto s3 = v.segment<2>(3);  // [x, 1]
  const Formula f1{s1 == s2};       // (x = 1) ∧ (1 = y)
  const Formula f2{s1 == s3};       // (x = x) ∧ (1 = 1) -> True

  ASSERT_TRUE(is_conjunction(f1));
  EXPECT_EQ(get_operands(f1).size(), 2);
  EXPECT_TRUE(is_true(f2));
}

TEST_F(SymbolicExpressionMatrixTest, ExpressionMatrixBlock) {
  Eigen::Matrix<Expression, 3, 3> m;
  // clang-format off
  m << x_, y_, z_,
       y_, 1, 2,
       z_, 3, 4;
  // clang-format on

  // b1 = [x, y]
  //      [y, 1]
  const auto b1 = m.block(0, 0, 2, 2);
  // b2 = [1, 2]
  //      [3, 4]
  const auto b2 = m.block<2, 2>(1, 1);
  // (x = 1) ∧ (y = 2) ∧ (y = 3) ∧ (1 = 4) -> False
  const Formula f{b1 == b2};

  EXPECT_TRUE(is_false(f));
}

// Checks relational operators (==, !=, <=, <, >=, >) between
// Matrix<Expression> and Matrix<Expression>.
TEST_F(SymbolicExpressionMatrixTest, MatrixExprRopMatrixExpr) {
  EXPECT_TRUE(CheckMatrixOperatorEq(A_, C_));
  EXPECT_TRUE(CheckMatrixOperatorEq(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckMatrixOperatorLte(A_, C_));
  EXPECT_TRUE(CheckMatrixOperatorLte(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckMatrixOperatorLt(A_, C_));
  EXPECT_TRUE(CheckMatrixOperatorLt(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckMatrixOperatorGte(A_, C_));
  EXPECT_TRUE(CheckMatrixOperatorGte(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckMatrixOperatorGt(A_, C_));
  EXPECT_TRUE(CheckMatrixOperatorGt(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(A_, C_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(B_ * A_, B_ * C_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between
// Matrix<Expression> and Matrix<Variable>
TEST_F(SymbolicExpressionMatrixTest, MatrixExprRopMatrixVar) {
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_expr_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_var_2_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_expr_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_var_2_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_expr_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_var_2_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_expr_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_var_2_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_expr_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_var_2_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_expr_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_var_2_, matrix_expr_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between
// Matrix<Expression> and Matrix<double>
TEST_F(SymbolicExpressionMatrixTest, MatrixExprRopMatrixDouble) {
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_expr_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_double_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_expr_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_double_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_expr_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_double_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_expr_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_double_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_expr_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_double_, matrix_expr_1_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_expr_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_double_, matrix_expr_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between
// Matrix<Variable> and Matrix<double>
TEST_F(SymbolicExpressionMatrixTest, MatrixVarRopMatrixDouble) {
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_var_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_double_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_var_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_double_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_var_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_double_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_var_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_double_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_var_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_double_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_var_1_, matrix_double_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_double_, matrix_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between
// Matrix<Variable> and Matrix<Variable>
TEST_F(SymbolicExpressionMatrixTest, MatrixVarRopMatrixVar) {
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_var_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorEq(matrix_var_2_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_var_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorLte(matrix_var_2_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_var_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorLt(matrix_var_2_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_var_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorGte(matrix_var_2_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_var_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorGt(matrix_var_2_, matrix_var_1_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_var_1_, matrix_var_2_));
  EXPECT_TRUE(CheckMatrixOperatorNeq(matrix_var_2_, matrix_var_1_));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

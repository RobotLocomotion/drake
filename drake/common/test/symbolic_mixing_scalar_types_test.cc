#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
namespace {

using std::ostringstream;
using std::string;

class SymbolicMixingScalarTypesTest : public ::testing::Test {
  template <typename Scalar>
  using Matrix2DontAlign = Eigen::Matrix<Scalar, 2, 2, Eigen::DontAlign>;
  template <typename Scalar>
  using Vector2DontAlign = Eigen::Matrix<Scalar, 2, 1, Eigen::DontAlign>;

 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variable var_w_{"w"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const Expression w_{var_w_};

  Matrix2DontAlign<Expression> M_expr_fixed_;
  MatrixX<Expression> M_expr_dyn_{2, 2};
  Vector2DontAlign<Expression> V_expr_fixed_;
  VectorX<Expression> V_expr_dyn_{2};

  Matrix2DontAlign<Variable> M_var_fixed_;
  MatrixX<Variable> M_var_dyn_{2, 2};
  Vector2DontAlign<Variable> V_var_fixed_;
  VectorX<Variable> V_var_dyn_{2};

  Matrix2DontAlign<double> M_double_fixed_;
  MatrixX<double> M_double_dyn_{2, 2};
  Vector2DontAlign<double> V_double_fixed_;
  VectorX<double> V_double_dyn_{2};

  void SetUp() override {
    // clang-format off
    M_expr_fixed_ << x_, y_,
                     z_, w_;
    M_expr_dyn_   << x_, y_,
                     z_, w_;
    V_expr_fixed_ << x_,
                     y_;
    V_expr_dyn_   << x_,
                     y_;
    // clang-format on

    // clang-format off
    M_var_fixed_ << var_x_, var_y_,
                    var_z_, var_w_;
    M_var_dyn_   << var_x_, var_y_,
                    var_z_, var_w_;
    V_var_fixed_ << var_x_,
                    var_y_;
    V_var_dyn_   << var_x_,
                    var_y_;
    // clang-format on

    // clang-format off
    M_double_fixed_ << 1, 2,
                       3, 4;
    M_double_dyn_   << 1, 2,
                       3, 4;
    V_double_fixed_ << 1,
                       2;
    V_double_dyn_   << 1,
                       2;
    // clang-format on
  }

  template <typename Scalar>
  string to_string(const Eigen::MatrixBase<Scalar>& m) {
    ostringstream oss;
    oss << m;
    return oss.str();
  }
};

TEST_F(SymbolicMixingScalarTypesTest, MatrixAdditionExprVar) {
  const MatrixX<Expression> M1{M_expr_fixed_ + M_var_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ + M_var_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ + M_var_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ + M_var_dyn_};
  const string expected{
      "      (2 * x)       (2 * y)\n      (2 * z)       (2 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixAdditionExprDouble) {
  const MatrixX<Expression> M1{M_expr_fixed_ + M_double_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ + M_double_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ + M_double_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ + M_double_dyn_};
  const string expected{
      "      (1 + x)       (2 + y)\n      (3 + z)       (4 + w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixAdditionVarExpr) {
  const MatrixX<Expression> M1{M_var_fixed_ + M_expr_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ + M_expr_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ + M_expr_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ + M_expr_dyn_};
  const string expected{
      "      (2 * x)       (2 * y)\n      (2 * z)       (2 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixAdditionVarDouble) {
  const MatrixX<Expression> M1{M_var_fixed_ + M_double_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ + M_double_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ + M_double_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ + M_double_dyn_};
  const string expected{
      "      (1 + x)       (2 + y)\n      (3 + z)       (4 + w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixAdditionDoubleExpr) {
  const MatrixX<Expression> M1{M_double_fixed_ + M_expr_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ + M_expr_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ + M_expr_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ + M_expr_dyn_};
  const string expected{
      "      (1 + x)       (2 + y)\n      (3 + z)       (4 + w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixAdditionDoubleVar) {
  const MatrixX<Expression> M1{M_double_fixed_ + M_var_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ + M_var_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ + M_var_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ + M_var_dyn_};
  const string expected{
      "      (1 + x)       (2 + y)\n      (3 + z)       (4 + w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixSubtractionExprVar) {
  const MatrixX<Expression> M1{M_expr_fixed_ - M_var_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ - M_var_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ - M_var_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ - M_var_dyn_};
  const string expected{"0 0\n0 0"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixSubtractionExprDouble) {
  const MatrixX<Expression> M1{M_expr_fixed_ - M_double_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ - M_double_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ - M_double_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ - M_double_dyn_};
  const string expected{
      "       (-1 + x)        (-2 + y)\n       (-3 + z)        (-4 + w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixSubtractionVarExpr) {
  const MatrixX<Expression> M1{M_var_fixed_ - M_expr_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ - M_expr_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ - M_expr_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ - M_expr_dyn_};
  const string expected{"0 0\n0 0"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixSubtractionVarDouble) {
  const MatrixX<Expression> M1{M_var_fixed_ - M_double_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ - M_double_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ - M_double_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ - M_double_dyn_};
  const string expected{
      "       (-1 + x)        (-2 + y)\n       (-3 + z)        (-4 + w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixSubtractionDoubleExpr) {
  const MatrixX<Expression> M1{M_double_fixed_ - M_expr_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ - M_expr_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ - M_expr_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ - M_expr_dyn_};
  const string expected{
      "      (1 - x)       (2 - y)\n      (3 - z)       (4 - w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixSubtractionDoubleVar) {
  const MatrixX<Expression> M1{M_double_fixed_ - M_var_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ - M_var_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ - M_var_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ - M_var_dyn_};
  const string expected{
      "      (1 - x)       (2 - y)\n      (3 - z)       (4 - w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixMatrixMultiplicationExprVar) {
  const MatrixX<Expression> M1{M_expr_fixed_ * M_var_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ * M_var_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ * M_var_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ * M_var_dyn_};
  const string expected{
      "                    ((y * z) + pow(x, 2))"
      "                     ((x * y) + (y * w))\n"
      "                    ((x * z) + (z * w))"
      "                     ((y * z) + pow(w, 2))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixVectorMultiplicationExprVar) {
  const MatrixX<Expression> M1{M_expr_fixed_ * V_var_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ * V_var_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ * V_var_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ * V_var_dyn_};
  const string expected{
      "                      (pow(x, 2) + pow(y, 2))\n"
      "                      ((x * z) + (y * w))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorMatrixMultiplicationExprVar) {
  const MatrixX<Expression> M1{V_expr_fixed_.transpose() * M_var_fixed_};
  const MatrixX<Expression> M2{V_expr_fixed_.transpose() * M_var_dyn_};
  const MatrixX<Expression> M3{V_expr_dyn_.transpose() * M_var_fixed_};
  const MatrixX<Expression> M4{V_expr_dyn_.transpose() * M_var_dyn_};
  const string expected{
      "                    ((y * z) + pow(x, 2))"
      "                     ((x * y) + (y * w))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorVectorMultiplicationExprVar) {
  const MatrixX<Expression> M1{V_expr_fixed_.transpose() * V_var_fixed_};
  const MatrixX<Expression> M2{V_expr_fixed_.transpose() * V_var_dyn_};
  const MatrixX<Expression> M3{V_expr_dyn_.transpose() * V_var_fixed_};
  const MatrixX<Expression> M4{V_expr_dyn_.transpose() * V_var_dyn_};
  const string expected{"                      (pow(x, 2) + pow(y, 2))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixMatrixMultiplicationVarExpr) {
  const MatrixX<Expression> M1{M_var_fixed_ * M_expr_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ * M_expr_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ * M_expr_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ * M_expr_dyn_};
  const string expected{
      "                    ((y * z) + pow(x, 2))"
      "                     ((x * y) + (y * w))\n"
      "                    ((x * z) + (z * w))"
      "                     ((y * z) + pow(w, 2))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixVectorMultiplicationVarExpr) {
  const MatrixX<Expression> M1{M_var_fixed_ * V_expr_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ * V_expr_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ * V_expr_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ * V_expr_dyn_};
  const string expected{
      "                      (pow(x, 2) + pow(y, 2))\n"
      "                      ((x * z) + (y * w))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorMatrixMultiplicationVarExpr) {
  const MatrixX<Expression> M1{V_var_fixed_.transpose() * M_expr_fixed_};
  const MatrixX<Expression> M2{V_var_fixed_.transpose() * M_expr_dyn_};
  const MatrixX<Expression> M3{V_var_dyn_.transpose() * M_expr_fixed_};
  const MatrixX<Expression> M4{V_var_dyn_.transpose() * M_expr_dyn_};
  const string expected{
      "                    ((y * z) + pow(x, 2))"
      "                     ((x * y) + (y * w))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorVectorMultiplicationVarExpr) {
  const MatrixX<Expression> M1{V_var_fixed_.transpose() * V_expr_fixed_};
  const MatrixX<Expression> M2{V_var_fixed_.transpose() * V_expr_dyn_};
  const MatrixX<Expression> M3{V_var_dyn_.transpose() * V_expr_fixed_};
  const MatrixX<Expression> M4{V_var_dyn_.transpose() * V_expr_dyn_};
  const string expected{"                      (pow(x, 2) + pow(y, 2))"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixMatrixMultiplicationExprDouble) {
  const MatrixX<Expression> M1{M_expr_fixed_ * M_double_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ * M_double_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ * M_double_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ * M_double_dyn_};
  const string expected{
      "              (x + 3 * y)"
      "               (2 * x + 4 * y)\n"
      "              (z + 3 * w)"
      "               (2 * z + 4 * w)"};
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixVectorMultiplicationExprDouble) {
  const MatrixX<Expression> M1{M_expr_fixed_ * V_double_fixed_};
  const MatrixX<Expression> M2{M_expr_fixed_ * V_double_dyn_};
  const MatrixX<Expression> M3{M_expr_dyn_ * V_double_fixed_};
  const MatrixX<Expression> M4{M_expr_dyn_ * V_double_dyn_};
  const string expected{"          (x + 2 * y)\n          (z + 2 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorMatrixMultiplicationExprDouble) {
  const MatrixX<Expression> M1{V_expr_fixed_.transpose() * M_double_fixed_};
  const MatrixX<Expression> M2{V_expr_fixed_.transpose() * M_double_dyn_};
  const MatrixX<Expression> M3{V_expr_dyn_.transpose() * M_double_fixed_};
  const MatrixX<Expression> M4{V_expr_dyn_.transpose() * M_double_dyn_};
  const string expected{
      "              (x + 3 * y)               (2 * x + 4 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorVectorMultiplicationExprDouble) {
  const MatrixX<Expression> M1{V_expr_fixed_.transpose() * V_double_fixed_};
  const MatrixX<Expression> M2{V_expr_fixed_.transpose() * V_double_dyn_};
  const MatrixX<Expression> M3{V_expr_dyn_.transpose() * V_double_fixed_};
  const MatrixX<Expression> M4{V_expr_dyn_.transpose() * V_double_dyn_};
  const string expected{"          (x + 2 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixMatrixMultiplicationDoubleExpr) {
  const MatrixX<Expression> M1{M_double_fixed_ * M_expr_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ * M_expr_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ * M_expr_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ * M_expr_dyn_};
  const string expected{
      "              (x + 2 * z)"
      "               (y + 2 * w)\n"
      "              (3 * x + 4 * z)"
      "               (3 * y + 4 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixVectorMultiplicationDoubleExpr) {
  const MatrixX<Expression> M1{M_double_fixed_ * V_expr_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ * V_expr_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ * V_expr_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ * V_expr_dyn_};
  const string expected{
      "              (x + 2 * y)\n              (3 * x + 4 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorMatrixMultiplicationDoubleExpr) {
  const MatrixX<Expression> M1{V_double_fixed_.transpose() * M_expr_fixed_};
  const MatrixX<Expression> M2{V_double_fixed_.transpose() * M_expr_dyn_};
  const MatrixX<Expression> M3{V_double_dyn_.transpose() * M_expr_fixed_};
  const MatrixX<Expression> M4{V_double_dyn_.transpose() * M_expr_dyn_};
  const string expected{"          (x + 2 * z)           (y + 2 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorVectorMultiplicationDoubleExpr) {
  const MatrixX<Expression> M1{V_double_fixed_.transpose() * V_expr_fixed_};
  const MatrixX<Expression> M2{V_double_fixed_.transpose() * V_expr_dyn_};
  const MatrixX<Expression> M3{V_double_dyn_.transpose() * V_expr_fixed_};
  const MatrixX<Expression> M4{V_double_dyn_.transpose() * V_expr_dyn_};
  const string expected{"          (x + 2 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixMatrixMultiplicationVarDouble) {
  const MatrixX<Expression> M1{M_var_fixed_ * M_double_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ * M_double_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ * M_double_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ * M_double_dyn_};
  const string expected{
      "              (x + 3 * y)"
      "               (2 * x + 4 * y)\n"
      "              (z + 3 * w)"
      "               (2 * z + 4 * w)"};
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixVectorMultiplicationVarDouble) {
  const MatrixX<Expression> M1{M_var_fixed_ * V_double_fixed_};
  const MatrixX<Expression> M2{M_var_fixed_ * V_double_dyn_};
  const MatrixX<Expression> M3{M_var_dyn_ * V_double_fixed_};
  const MatrixX<Expression> M4{M_var_dyn_ * V_double_dyn_};
  const string expected{"          (x + 2 * y)\n          (z + 2 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorMatrixMultiplicationVarDouble) {
  const MatrixX<Expression> M1{V_var_fixed_.transpose() * M_double_fixed_};
  const MatrixX<Expression> M2{V_var_fixed_.transpose() * M_double_dyn_};
  const MatrixX<Expression> M3{V_var_dyn_.transpose() * M_double_fixed_};
  const MatrixX<Expression> M4{V_var_dyn_.transpose() * M_double_dyn_};
  const string expected{
      "              (x + 3 * y)               (2 * x + 4 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorVectorMultiplicationVarDouble) {
  const MatrixX<Expression> M1{V_var_fixed_.transpose() * V_double_fixed_};
  const MatrixX<Expression> M2{V_var_fixed_.transpose() * V_double_dyn_};
  const MatrixX<Expression> M3{V_var_dyn_.transpose() * V_double_fixed_};
  const MatrixX<Expression> M4{V_var_dyn_.transpose() * V_double_dyn_};
  const string expected{"          (x + 2 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixMatrixMultiplicationDoubleVar) {
  const MatrixX<Expression> M1{M_double_fixed_ * M_var_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ * M_var_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ * M_var_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ * M_var_dyn_};
  const string expected{
      "              (x + 2 * z)"
      "               (y + 2 * w)\n"
      "              (3 * x + 4 * z)"
      "               (3 * y + 4 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, MatrixVectorMultiplicationDoubleVar) {
  const MatrixX<Expression> M1{M_double_fixed_ * V_var_fixed_};
  const MatrixX<Expression> M2{M_double_fixed_ * V_var_dyn_};
  const MatrixX<Expression> M3{M_double_dyn_ * V_var_fixed_};
  const MatrixX<Expression> M4{M_double_dyn_ * V_var_dyn_};
  const string expected{
      "              (x + 2 * y)\n              (3 * x + 4 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorMatrixMultiplicationDoubleVar) {
  const MatrixX<Expression> M1{V_double_fixed_.transpose() * M_var_fixed_};
  const MatrixX<Expression> M2{V_double_fixed_.transpose() * M_var_dyn_};
  const MatrixX<Expression> M3{V_double_dyn_.transpose() * M_var_fixed_};
  const MatrixX<Expression> M4{V_double_dyn_.transpose() * M_var_dyn_};
  const string expected{"          (x + 2 * z)           (y + 2 * w)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}

TEST_F(SymbolicMixingScalarTypesTest, VectorVectorMultiplicationDoubleVar) {
  const MatrixX<Expression> M1{V_double_fixed_.transpose() * V_var_fixed_};
  const MatrixX<Expression> M2{V_double_fixed_.transpose() * V_var_dyn_};
  const MatrixX<Expression> M3{V_double_dyn_.transpose() * V_var_fixed_};
  const MatrixX<Expression> M4{V_double_dyn_.transpose() * V_var_dyn_};
  const string expected{"          (x + 2 * y)"};
  EXPECT_EQ(to_string(M1), expected);
  EXPECT_EQ(to_string(M2), expected);
  EXPECT_EQ(to_string(M3), expected);
  EXPECT_EQ(to_string(M4), expected);
}
}  // namespace
}  // namespace symbolic
}  // namespace drake

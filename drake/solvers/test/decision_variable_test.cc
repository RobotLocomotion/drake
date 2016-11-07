#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
template <typename Derived>
void CheckValue(const DecisionVariableMatrix& mat,
                const Eigen::MatrixBase<Derived>& mat_value) {
  EXPECT_EQ(mat.rows(), mat_value.rows());
  EXPECT_EQ(mat.cols(), mat_value.cols());
  EXPECT_TRUE(CompareMatrices(mat.value(), mat_value, 1E-10,
                              MatrixCompareType::absolute));
  // Check value(i, j).
  for (int i = 0; i < static_cast<int>(mat.rows()); ++i) {
    for (int j = 0; j < static_cast<int>(mat.cols()); ++j) {
      EXPECT_DOUBLE_EQ(mat.value(i, j), mat_value(i, j));
    }
  }
  // Check row(i).
  for (int i = 0; i < static_cast<int>(mat.rows()); ++i) {
    EXPECT_TRUE(CompareMatrices(mat.row(i).value(), mat_value.row(i), 1E-10,
                                MatrixCompareType::absolute));
  }
  // Check col(j).
  for (int j = 0; j < static_cast<int>(mat.cols()); ++j) {
    EXPECT_TRUE(CompareMatrices(mat.col(j).value(), mat_value.col(j), 1E-10,
                                MatrixCompareType::absolute));
  }

  // Check block(row_start, col_start, rows, cols).
  for (int row_start = 0; row_start < static_cast<int>(mat.rows());
       ++row_start) {
    for (int col_start = 0; col_start < static_cast<int>(mat.cols());
         ++col_start) {
      for (int rows = 1; row_start + rows < static_cast<int>(mat.rows());
           ++rows) {
        for (int cols = 1; col_start + cols < static_cast<int>(mat.cols());
             ++cols) {
          EXPECT_TRUE(CompareMatrices(
              mat.block(row_start, col_start, rows, cols).value(),
              mat_value.block(row_start, col_start, rows, cols), 1E-10,
              MatrixCompareType::absolute));
        }
      }
    }
  }
}

GTEST_TEST(TestDecisionVariable, TestDecisionVariableValue) {
  MathematicalProgram prog;
  auto X = prog.AddContinuousVariables(2, 3, std::vector<std::string>(6, "X"));
  EXPECT_EQ(prog.num_vars(), 6);
  auto S =
      prog.AddSymmetricContinuousVariables(3, std::vector<std::string>(6, "S"));
  EXPECT_EQ(prog.num_vars(), 12);
  auto x = prog.AddContinuousVariables(6, "x");
  EXPECT_EQ(prog.num_vars(), 18);
  Eigen::Matrix<double, 6, 1> x_value;
  x_value << 0, 2, 4, 6, 8, 10;
  Eigen::Matrix<double, 6, 1> s_value;
  s_value << 0, -2, -4, -6, -8, -10;
  Eigen::Matrix<double, 3, 3> S_expected;
  S_expected << 0, -2, -4, -2, -6, -8, -4, -8, -10;
  Eigen::Matrix<double, 18, 1> var_values;
  var_values << x_value, s_value, x_value;
  prog.SetDecisionVariableValues(var_values);
  Eigen::MatrixXd X_expected = x_value;
  X_expected.resize(2, 3);

  CheckValue(X, X_expected);
  CheckValue(S, S_expected);
  CheckValue(x, x_value);

  // Check head(i), tail(i) and segment(i, j).
  for (int i = 0; i < static_cast<int>(x.rows()); ++i) {
    EXPECT_TRUE(CompareMatrices(x.head(i).value(), x_value.head(i), 1E-10,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(x.tail(i).value(), x_value.tail(i), 1E-10,
                                MatrixCompareType::absolute));
    for (int j = 1; i + j < static_cast<int>(x.rows()); ++j) {
      EXPECT_TRUE(CompareMatrices(x.segment(i, j).value(),
                                  x_value.segment(i, j), 1E-10,
                                  MatrixCompareType::absolute));
    }
  }
}
}  // namespace solvers
}  // namespace drake

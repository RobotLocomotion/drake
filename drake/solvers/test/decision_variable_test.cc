#include "drake/solvers/decision_variable.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
template <typename Derived>
typename std::enable_if<
    std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
    bool>::type
MatrixDecisionVariableContainsIndex(const MathematicalProgram& prog,
                                    const Eigen::MatrixBase<Derived>& v,
                                    size_t index) {
  static_assert(
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      "The input should be a matrix of symbolic::Variable.");
  for (int i = 0; i < v.rows(); ++i) {
    for (int j = 0; j < v.cols(); ++j) {
      if (prog.FindDecisionVariableIndex(v(i, j)) == index) {
        return true;
      }
    }
  }
  return false;
}

template <typename Derived>
bool CheckDecisionVariableType(const MathematicalProgram& prog,
                               const Eigen::MatrixBase<Derived>& var,
                               MathematicalProgram::VarType type_expected) {
  const auto& variable_types = prog.DecisionVariableTypes();
  for (int i = 0; i < var.rows(); ++i) {
    for (int j = 0; j < var.cols(); ++j) {
      if ((variable_types[prog.FindDecisionVariableIndex(var(i, j))] !=
           type_expected) ||
          (prog.DecisionVariableType(var(i, j)) != type_expected)) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

/*
* Test adding decision variables.
*/
GTEST_TEST(TestDecisionVariable, TestDecisionVariableValue) {
  MathematicalProgram prog;
  auto X1 = prog.NewContinuousVariables(2, 3, "X");
  static_assert(decltype(X1)::RowsAtCompileTime == Eigen::Dynamic &&
                    decltype(X1)::ColsAtCompileTime == Eigen::Dynamic,
                "should be a dynamic sized matrix");
  std::stringstream msg_buff1;
  msg_buff1 << X1 << std::endl;
  EXPECT_EQ(msg_buff1.str(), "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n");
  EXPECT_EQ(prog.num_vars(), 6u);
  EXPECT_FALSE(math::IsSymmetric(X1));

  auto S1 = prog.NewSymmetricContinuousVariables(3, "S");
  static_assert(decltype(S1)::RowsAtCompileTime == Eigen::Dynamic &&
                    decltype(S1)::ColsAtCompileTime == Eigen::Dynamic,
                "should be a dynamic sized matrix");
  std::stringstream msg_buff2;
  msg_buff2 << S1 << std::endl;
  EXPECT_EQ(
      msg_buff2.str(),
      "S(0,0) S(1,0) S(2,0)\nS(1,0) S(1,1) S(2,1)\nS(2,0) S(2,1) S(2,2)\n");
  EXPECT_EQ(prog.num_vars(), 12u);
  EXPECT_TRUE(math::IsSymmetric(S1));
  auto x1 = prog.NewContinuousVariables(6, "x");
  static_assert(decltype(x1)::RowsAtCompileTime == Eigen::Dynamic &&
                    decltype(x1)::ColsAtCompileTime == 1,
                "should be a dynamic sized matrix");
  std::stringstream msg_buff3;
  msg_buff3 << x1 << std::endl;
  EXPECT_EQ(msg_buff3.str(), "x(0)\nx(1)\nx(2)\nx(3)\nx(4)\nx(5)\n");
  EXPECT_EQ(prog.num_vars(), 18u);
  EXPECT_FALSE(math::IsSymmetric(x1));
  std::array<std::string, 6> X_name = {{"X1", "X2", "X3", "X4", "X5", "X6"}};
  auto X2 = prog.NewContinuousVariables<2, 3>(X_name);
  std::stringstream msg_buff4;
  msg_buff4 << X2 << std::endl;
  EXPECT_EQ(msg_buff4.str(), "X1 X3 X5\nX2 X4 X6\n");
  static_assert(decltype(X2)::RowsAtCompileTime == 2 &&
                    decltype(X2)::ColsAtCompileTime == 3,
                "should be a static matrix of type 2 x 3");
  EXPECT_EQ(prog.num_vars(), 24u);
  EXPECT_FALSE(math::IsSymmetric(X2));
  auto b1 = prog.NewBinaryVariables(6, "b1");
  std::stringstream msg_buff5;
  msg_buff5 << b1 << std::endl;
  EXPECT_EQ(msg_buff5.str(), "b1(0)\nb1(1)\nb1(2)\nb1(3)\nb1(4)\nb1(5)\n");

  // Tests setting values for the decision variables.
  Eigen::Matrix<double, 6, 1> x_value;
  x_value << 0, 2, 4, 6, 8, 10;
  Eigen::Matrix<double, 6, 1> s_value;
  s_value << 0, -2, -4, -6, -8, -10;
  Eigen::Matrix<double, 3, 3> S_expected;
  S_expected << 0, -2, -4, -2, -6, -8, -4, -8, -10;
  Eigen::Matrix<double, 6, 1> b_value;
  b_value << 0, 1, 1, 1, 0, 0;
  Eigen::Matrix<double, 30, 1> var_values;
  var_values << x_value, s_value, x_value, x_value, b_value;
  prog.SetDecisionVariableValues(var_values);
  Eigen::MatrixXd X_expected = x_value;
  X_expected.resize(2, 3);
  Eigen::MatrixXd b_expected = b_value;

  // Tests if the values in the decision variables are correct.
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(X1), X_expected, 1E-14,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(S1), S_expected, 1E-14,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(x1), x_value, 1E-14,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(X2), X_expected, 1E-14,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(b1), b_expected, 1E-14,
                              MatrixCompareType::absolute));

  // Tests if the variable type is correct.
  EXPECT_TRUE(CheckDecisionVariableType(
      prog, X1, MathematicalProgram::VarType::CONTINUOUS));
  EXPECT_TRUE(CheckDecisionVariableType(
      prog, S1, MathematicalProgram::VarType::CONTINUOUS));
  EXPECT_TRUE(CheckDecisionVariableType(
      prog, x1, MathematicalProgram::VarType::CONTINUOUS));
  EXPECT_TRUE(CheckDecisionVariableType(
      prog, X2, MathematicalProgram::VarType::CONTINUOUS));
  EXPECT_TRUE(CheckDecisionVariableType(prog, b1,
                                        MathematicalProgram::VarType::BINARY));

  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(MatrixDecisionVariableContainsIndex(prog, X1, i));
    EXPECT_TRUE(MatrixDecisionVariableContainsIndex(prog, S1, i + 6));
    EXPECT_TRUE(MatrixDecisionVariableContainsIndex(prog, x1, i + 12));
    EXPECT_TRUE(MatrixDecisionVariableContainsIndex(prog, X2, i + 18));
  }

  // Tests if all entries in x1 are unique, that x1(i) = x1(j) iff i = j.
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_EQ(x1(i) == x1(j), i == j);
    }
  }

  // Tests concatenating two Eigen matrices of symbolic variables.
  MatrixDecisionVariable<2, 6> X_assembled;
  X_assembled << X1, X2;
  Eigen::Matrix<double, 2, 6> X_assembled_expected;
  X_assembled_expected << X_expected, X_expected;
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(X_assembled),
                              X_assembled_expected, 1E-10,
                              MatrixCompareType::absolute));

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_TRUE(X_assembled(i, j) == X1(i, j));
      EXPECT_TRUE(X_assembled(i, j + 3) == X2(i, j));
    }
  }

  std::unordered_set<symbolic::Variable, drake::hash_value<symbolic::Variable>>
      X1_unique_variables_expected;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      X1_unique_variables_expected.insert(X1(i, j));
    }
  }
}

GTEST_TEST(TestDecisionVariable, TestVariableListRef) {
  symbolic::Variable x1("x1");
  symbolic::Variable x2("x2");
  symbolic::Variable x3("x3");
  symbolic::Variable x4("x4");

  VectorDecisionVariable<2> x_vec1(x3, x1);
  VectorDecisionVariable<2> x_vec2(x2, x4);
  VariableRefList var_list{x_vec1, x_vec2};

  VectorXDecisionVariable stacked_vars = ConcatenateVariableRefList(var_list);
  EXPECT_EQ(stacked_vars.rows(), 4);
  EXPECT_EQ(stacked_vars(0), x3);
  EXPECT_EQ(stacked_vars(1), x1);
  EXPECT_EQ(stacked_vars(2), x2);
  EXPECT_EQ(stacked_vars(3), x4);
}
}  // namespace solvers
}  // namespace drake

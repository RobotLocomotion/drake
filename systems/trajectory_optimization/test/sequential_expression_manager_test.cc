#include "drake/systems/trajectory_optimization/sequential_expression_manager.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace internal {

using std::string;
using std::vector;
using symbolic::Expression;
using symbolic::Substitution;
using symbolic::Variable;

class SequentialExpressionManagerTests : public testing::Test {
 public:
  static MatrixX<Variable> MakeVariableMatrix(int rows, int cols) {
    MatrixX<Variable> variable{rows, cols};
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        variable(i, j) = Variable(fmt::format("x_{}_{}", i, j));
      }
    }
    return variable;
  }

 protected:
  const int num_variables_{3};
  const int num_samples_{5};
  SequentialExpressionManager dut_{num_samples_};
};

// Verify that num_samples() works as expected.
TEST_F(SequentialExpressionManagerTests, NumSamplesTest) {
  EXPECT_EQ(dut_.num_samples(), num_samples_);
}

// Test with a matrix of Expressions.
TEST_F(SequentialExpressionManagerTests, RegisterAndSubstituteExpressionTest) {
  MatrixX<Expression> x_sequential =
      MakeVariableMatrix(num_variables_, num_samples_);
  x_sequential.cwiseSqrt();
  VectorX<Variable> x_placeholder =
      dut_.RegisterSequentialExpressions(x_sequential.cast<Expression>(), "x");
  EXPECT_EQ(dut_.num_rows("x"), num_variables_);
  MatrixX<Expression> placeholder_expression =
      x_placeholder * x_placeholder.transpose();
  for (int j = 0; j < num_samples_; ++j) {
    MatrixX<Expression> expected_expression =
        x_sequential.col(j) * x_sequential.col(j).transpose();
    Substitution subs = dut_.ConstructPlaceholderVariableSubstitution(j);
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

TEST_F(SequentialExpressionManagerTests,
       RegisterAndSubstituteExpressionTestWithPlaceholders) {
  const MatrixX<Expression> x_sequential{
      MakeVariableMatrix(num_variables_, num_samples_)};
  x_sequential.cwiseSqrt();
  const VectorX<Variable> x_placeholder{
      symbolic::MakeVectorContinuousVariable(num_variables_, "x")};
  dut_.RegisterSequentialExpressions(x_placeholder,
                                     x_sequential.cast<Expression>(), "x");
  EXPECT_EQ(dut_.num_rows("x"), num_variables_);
  const MatrixX<Expression> placeholder_expression{x_placeholder *
                                                   x_placeholder.transpose()};
  for (int j = 0; j < num_samples_; ++j) {
    const MatrixX<Expression> expected_expression{
        x_sequential.col(j) * x_sequential.col(j).transpose()};
    const Substitution subs{dut_.ConstructPlaceholderVariableSubstitution(j)};
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

// Test with an Eigen::Map applied to a vector of Expressions.
TEST_F(SequentialExpressionManagerTests, RegisterAndSubstituteMapTest) {
  VectorX<Expression> x_sequential =
      MakeVariableMatrix(num_variables_ * num_samples_, 1);
  VectorX<Variable> x_placeholder = dut_.RegisterSequentialExpressions(
      Eigen::Map<MatrixX<Expression>>(x_sequential.data(), num_variables_,
                                      num_samples_),
      "x");
  MatrixX<Expression> placeholder_expression =
      x_placeholder * x_placeholder.transpose();
  for (int j = 0; j < num_samples_; ++j) {
    MatrixX<Expression> expected_expression =
        x_sequential.segment(j * num_variables_, num_variables_) *
        x_sequential.segment(j * num_variables_, num_variables_).transpose();
    Substitution subs = dut_.ConstructPlaceholderVariableSubstitution(j);
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

// Test with an Eigen::Map and cast() applied to a vector of Variables.
TEST_F(SequentialExpressionManagerTests, RegisterAndSubstituteMapCastTest) {
  VectorX<Variable> x_sequential =
      MakeVariableMatrix(num_variables_ * num_samples_, 1);
  VectorX<Variable> x_placeholder = dut_.RegisterSequentialExpressions(
      Eigen::Map<MatrixX<Variable>>(x_sequential.data(), num_variables_,
                                    num_samples_)
          .cast<Expression>(),
      "x");
  MatrixX<Expression> placeholder_expression =
      x_placeholder * x_placeholder.transpose();
  for (int j = 0; j < num_samples_; ++j) {
    MatrixX<Expression> expected_expression =
        x_sequential.segment(j * num_variables_, num_variables_) *
        x_sequential.segment(j * num_variables_, num_variables_).transpose();
    Substitution subs = dut_.ConstructPlaceholderVariableSubstitution(j);
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

// Verify that GetSequentialExpressionsByName behaves as expected.
TEST_F(SequentialExpressionManagerTests, GetSequentialExpressionsByNameTest) {
  MatrixX<Expression> x_sequential =
      MakeVariableMatrix(num_variables_, num_samples_);
  x_sequential.cwiseSqrt();
  dut_.RegisterSequentialExpressions(x_sequential.cast<Expression>(), "x");
  for (int j = 0; j < num_samples_; ++j) {
    VectorX<Expression> expected_expression = x_sequential.col(j);
    EXPECT_EQ(dut_.GetSequentialExpressionsByName("x", j), expected_expression);
  }
}

TEST_F(SequentialExpressionManagerTests, GetSequentialExpressionNamesTest) {
  MatrixX<Variable> x_sequential =
      MakeVariableMatrix(num_variables_, num_samples_);
  VectorX<Variable> x_placeholder = dut_.RegisterSequentialExpressions(
      Eigen::Map<MatrixX<Variable>>(x_sequential.data(), num_variables_,
                                    num_samples_)
          .cast<Expression>(),
      "x");

  const vector<string> seq_variable_names = dut_.GetSequentialExpressionNames();
  EXPECT_EQ(seq_variable_names.size(), num_variables_ * num_samples_);

  auto check_name = [](const vector<string> vec, const string& name) {
    return find(vec.begin(), vec.end(), name) != vec.end();
  };

  for (int i = 0; i < num_variables_; ++i) {
    for (int j = 0; j < num_samples_; ++j) {
      EXPECT_TRUE(check_name(seq_variable_names, fmt::format("x_{}_{}", i, j)));
    }
  }
}

}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake

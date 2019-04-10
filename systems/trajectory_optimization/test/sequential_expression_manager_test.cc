#include "drake/systems/trajectory_optimization/sequential_expression_manager.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace internal {

using symbolic::Expression;
using symbolic::Substitution;
using symbolic::Variable;

class SequentialExpressionManagerTests : public testing::Test {
 public:
  MatrixX<Variable> MakeVariableMatrix(int rows, int cols) const {
    MatrixX<Variable> variable{rows, cols};
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        variable(i, j) = Variable(fmt::format("x_{}_{}", i, j));
      }
    }
    return variable;
  }

 protected:
  const int num_variables{3};
  const int num_samples{5};
  SequentialExpressionManager dut{num_samples};
};

// Verify that num_samples() works as expected.
TEST_F(SequentialExpressionManagerTests, NumSamplesTest) {
  EXPECT_EQ(dut.num_samples(), num_samples);
}

// Test with a matrix of Expressions.
TEST_F(SequentialExpressionManagerTests, RegisterAndSubstituteExpressionTest) {
  MatrixX<Expression> x_sequential =
      MakeVariableMatrix(num_variables, num_samples);
  x_sequential.cwiseSqrt();
  VectorX<Variable> x_placeholder =
      dut.RegisterSequentialExpressions(x_sequential.cast<Expression>(), "x");
  MatrixX<Expression> placeholder_expression =
      x_placeholder * x_placeholder.transpose();
  for (int j = 0; j < num_samples; ++j) {
    MatrixX<Expression> expected_expression =
        x_sequential.col(j) * x_sequential.col(j).transpose();
    Substitution subs = dut.ConstructPlaceholderVariableSubstitution(j);
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

// Test with an Eigen::Map applied to a vector of Expressions.
TEST_F(SequentialExpressionManagerTests, RegisterAndSubstituteMapTest) {
  VectorX<Expression> x_sequential =
      MakeVariableMatrix(num_variables * num_samples, 1);
  VectorX<Variable> x_placeholder = dut.RegisterSequentialExpressions(
      Eigen::Map<MatrixX<Expression>>(x_sequential.data(), num_variables,
                                      num_samples),
      "x");
  MatrixX<Expression> placeholder_expression =
      x_placeholder * x_placeholder.transpose();
  for (int j = 0; j < num_samples; ++j) {
    MatrixX<Expression> expected_expression =
        x_sequential.segment(j * num_variables, num_variables) *
        x_sequential.segment(j * num_variables, num_variables).transpose();
    Substitution subs = dut.ConstructPlaceholderVariableSubstitution(j);
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

// Test with an Eigen::Map and cast() applied to a vector of Variables.
TEST_F(SequentialExpressionManagerTests, RegisterAndSubstituteMapCastTest) {
  VectorX<Variable> x_sequential =
      MakeVariableMatrix(num_variables * num_samples, 1);
  VectorX<Variable> x_placeholder = dut.RegisterSequentialExpressions(
      Eigen::Map<MatrixX<Variable>>(x_sequential.data(), num_variables,
                                    num_samples)
          .cast<Expression>(),
      "x");
  MatrixX<Expression> placeholder_expression =
      x_placeholder * x_placeholder.transpose();
  for (int j = 0; j < num_samples; ++j) {
    MatrixX<Expression> expected_expression =
        x_sequential.segment(j * num_variables, num_variables) *
        x_sequential.segment(j * num_variables, num_variables).transpose();
    Substitution subs = dut.ConstructPlaceholderVariableSubstitution(j);
    EXPECT_EQ(symbolic::Substitute(placeholder_expression, subs),
              expected_expression);
  }
}

// Verify that GetSequentialExpressionsByName behaves as expected.
TEST_F(SequentialExpressionManagerTests, GetSequentialExpressionsByNameTest) {
  MatrixX<Expression> x_sequential =
      MakeVariableMatrix(num_variables, num_samples);
  x_sequential.cwiseSqrt();
  VectorX<Variable> x_placeholder =
      dut.RegisterSequentialExpressions(x_sequential.cast<Expression>(), "x");
  for (int j = 0; j < num_samples; ++j) {
    VectorX<Expression> expected_expression = x_sequential.col(j);
    EXPECT_EQ(dut.GetSequentialExpressionsByName("x", j), expected_expression);
  }
}
}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake

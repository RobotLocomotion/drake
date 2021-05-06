#include "drake/solvers/create_cost.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities//expect_throws_message.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace solvers {
namespace internal {
class CreateCostTest : public ::testing::Test {
 protected:
  const symbolic::Variable x_{"x"};
  const symbolic::Variable y_{"y"};
  const symbolic::Variable z_{"z"};
};

TEST_F(CreateCostTest, CreateQuadraticCostConvex) {
  const symbolic::Expression e1(x_ * x_ + 4 * y_ * y_ + 3 * x_ * y_ + 2 * x_ +
                                1);
  const auto constraint1 =
      ParseQuadraticCost(e1, false /* allow_nonconvex=false */);
  EXPECT_FALSE(constraint1.evaluator()->allow_nonconvex());
  EXPECT_EQ(constraint1.variables().rows(), 2);
  if (constraint1.variables()(0).get_id() == x_.get_id()) {
    EXPECT_PRED2(symbolic::test::VarEqual, constraint1.variables()(1), y_);
    EXPECT_TRUE(CompareMatrices(constraint1.evaluator()->Q(),
                                (Eigen::Matrix2d() << 2, 3, 3, 8).finished()));
    EXPECT_TRUE(
        CompareMatrices(constraint1.evaluator()->b(), Eigen::Vector2d(2, 0)));
  } else if (constraint1.variables()(0).get_id() == y_.get_id()) {
    EXPECT_PRED2(symbolic::test::VarEqual, constraint1.variables()(1), x_);
    EXPECT_TRUE(CompareMatrices(constraint1.evaluator()->Q(),
                                (Eigen::Matrix2d() << 8, 3, 3, 2).finished()));
    EXPECT_TRUE(
        CompareMatrices(constraint1.evaluator()->b(), Eigen::Vector2d(0, 2)));
  }
  EXPECT_EQ(constraint1.evaluator()->c(), 1);
}

TEST_F(CreateCostTest, CreateQuadraticCostNonConvex) {
  const symbolic::Expression e(x_ * x_ + 4 * y_ * y_ + 5 * x_ * y_ + 2 * x_ +
                               1);
  const auto constraint = ParseQuadraticCost(e, true /* allow_nonconvex=true*/);
  EXPECT_TRUE(constraint.evaluator()->allow_nonconvex());
  EXPECT_EQ(constraint.variables().rows(), 2);
  if (constraint.variables()(0).get_id() == x_.get_id()) {
    EXPECT_PRED2(symbolic::test::VarEqual, constraint.variables()(1), y_);
    EXPECT_TRUE(CompareMatrices(constraint.evaluator()->Q(),
                                (Eigen::Matrix2d() << 2, 5, 5, 8).finished()));
    EXPECT_TRUE(
        CompareMatrices(constraint.evaluator()->b(), Eigen::Vector2d(2, 0)));
  } else if (constraint.variables()(0).get_id() == y_.get_id()) {
    EXPECT_PRED2(symbolic::test::VarEqual, constraint.variables()(1), x_);
    EXPECT_TRUE(CompareMatrices(constraint.evaluator()->Q(),
                                (Eigen::Matrix2d() << 8, 5, 5, 2).finished()));
    EXPECT_TRUE(
        CompareMatrices(constraint.evaluator()->b(), Eigen::Vector2d(0, 2)));
  }
  EXPECT_EQ(constraint.evaluator()->c(), 1);

  // Now parse the cost with allow_nonconvex=false. Expect to throw an error.
  DRAKE_EXPECT_THROWS_MESSAGE(ParseQuadraticCost(e, false),
                              std::invalid_argument,
                              ".* the input Hessian matrix .*");
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake

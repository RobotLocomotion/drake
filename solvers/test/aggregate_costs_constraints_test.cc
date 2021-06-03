#include "drake/solvers/aggregate_costs_constraints.h"

#include <limits>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

using ::testing::HasSubstr;

namespace drake {
namespace solvers {
const double kInf = std::numeric_limits<double>::infinity();
class TestAggregateCostsAndConstraints : public ::testing::Test {
 public:
  TestAggregateCostsAndConstraints() {
    for (int i = 0; i < 4; ++i) {
      x_.push_back(symbolic::Variable(fmt::format("x{}", i)));
    }
  }

 protected:
  std::vector<symbolic::Variable> x_;
};

TEST_F(TestAggregateCostsAndConstraints, TestQuadraticCostsOnly) {
  // Test a single quadratic cost.
  std::vector<Binding<QuadraticCost>> quadratic_costs;
  Eigen::Matrix2d Q1;
  // clang-format off
  Q1 << 1, 2,
        2, 3;
  // clang-format on
  const Eigen::Vector2d b1(5, 0.);
  const double c1{2};
  const Vector2<symbolic::Variable> vars1(x_[0], x_[2]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q1, b1, c1),
                               vars1);

  Eigen::SparseMatrix<double> Q_lower;
  Eigen::SparseVector<double> linear_coeff;
  VectorX<symbolic::Variable> quadratic_vars;
  VectorX<symbolic::Variable> linear_vars;
  double constant_cost;
  AggregateQuadraticAndLinearCosts(quadratic_costs, {}, &Q_lower,
                                   &quadratic_vars, &linear_coeff, &linear_vars,
                                   &constant_cost);
  EXPECT_EQ(quadratic_vars.rows(), 2);
  EXPECT_EQ(quadratic_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(quadratic_vars(1).get_id(), x_[2].get_id());
  EXPECT_TRUE(
      CompareMatrices(Eigen::MatrixXd(Q_lower),
                      Eigen::MatrixXd(Q1.triangularView<Eigen::Lower>())));
  EXPECT_TRUE(CompareMatrices(Eigen::VectorXd(linear_coeff), Vector1d(5)));
  EXPECT_EQ(linear_vars.rows(), 1);
  EXPECT_EQ(linear_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(constant_cost, c1);

  // Now test multiple costs. These costs share variables.
  Eigen::Matrix3d Q2;
  // clang-format off
  Q2 << 10, 20, 30,
        20, 40,  0,
        30,  0, 50;
  // clang-format on
  const Eigen::Vector3d b2(10, 0, 50);
  const double c2 = 40;
  const Vector3<symbolic::Variable> vars2(x_[2], x_[1], x_[3]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q2, b2, c2),
                               vars2);
  AggregateQuadraticAndLinearCosts(quadratic_costs, {}, &Q_lower,
                                   &quadratic_vars, &linear_coeff, &linear_vars,
                                   &constant_cost);
  EXPECT_EQ(quadratic_vars.rows(), 4);
  EXPECT_EQ(quadratic_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(quadratic_vars(1).get_id(), x_[2].get_id());
  EXPECT_EQ(quadratic_vars(2).get_id(), x_[1].get_id());
  EXPECT_EQ(quadratic_vars(3).get_id(), x_[3].get_id());
  Eigen::Matrix4d Q_expected;
  // [x(0); x(2)] * Q1 * [x0; x(2)] + [x(2), x(1), x(3)] * Q2 * [x(2), x(1),
  // x(3)] = [x(0); x(2); x(1); x(3)] * Q_expected * [x(0); x(2); x(1); x(3)]
  // clang-format off
  Q_expected << 1,  0,  0,  0,
                2, 13,  0,  0,
                0, 20, 40,  0,
                0, 30,  0, 50;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(Q_lower), Q_expected));
  EXPECT_TRUE(CompareMatrices(Eigen::VectorXd(linear_coeff),
                              Eigen::Vector3d(5, 10, 50)));
  EXPECT_EQ(linear_vars.rows(), 3);
  EXPECT_EQ(linear_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(linear_vars(1).get_id(), x_[2].get_id());
  EXPECT_EQ(linear_vars(2).get_id(), x_[3].get_id());
  EXPECT_EQ(constant_cost, c1 + c2);

  // Add another quadratic cost.
  Eigen::Matrix2d Q3;
  Q3 << 0, 0, 0, 1;
  const Eigen::Vector2d b3(0.1, 0.5);
  const double c3 = 0.5;
  const Vector2<symbolic::Variable> vars3(x_[2], x_[1]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q3, b3, c3),
                               vars3);
  AggregateQuadraticAndLinearCosts(quadratic_costs, {}, &Q_lower,
                                   &quadratic_vars, &linear_coeff, &linear_vars,
                                   &constant_cost);
  // Check if the aggregated cost (in symbolic form) is correct.
  EXPECT_PRED2(
      symbolic::test::ExprEqual,
      quadratic_vars
          .dot(Eigen::MatrixXd(
                   Eigen::MatrixXd(Q_lower).selfadjointView<Eigen::Lower>()) *
               quadratic_vars)
          .Expand(),
      (vars1.dot(Q1 * vars1) + vars2.dot(Q2 * vars2) + vars3.dot(Q3 * vars3))
          .Expand());

  EXPECT_PRED2(symbolic::test::ExprEqual,
               Eigen::VectorXd(linear_coeff).dot(linear_vars),
               (b1.dot(vars1) + b2.dot(vars2) + b3.dot(vars3)).Expand());
  EXPECT_EQ(constant_cost, c1 + c2 + c3);
}

TEST_F(TestAggregateCostsAndConstraints, LinearCostsOnly) {
  // Test a single cost. This cost has a sparse linear coefficient.
  std::vector<Binding<LinearCost>> linear_costs;
  const Eigen::Vector3d a1(1, 0, 2);
  const double b1{3};
  const Vector3<symbolic::Variable> var1(x_[0], x_[2], x_[1]);
  linear_costs.emplace_back(std::make_shared<LinearCost>(a1, b1), var1);
  Eigen::SparseVector<double> linear_coeff;
  VectorX<symbolic::Variable> vars;
  double constant_cost;
  AggregateLinearCosts(linear_costs, &linear_coeff, &vars, &constant_cost);
  EXPECT_TRUE(
      CompareMatrices(Eigen::VectorXd(linear_coeff), Eigen::Vector2d(1, 2)));
  EXPECT_EQ(vars.rows(), 2);
  EXPECT_EQ(vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(vars(1).get_id(), x_[1].get_id());
  EXPECT_EQ(constant_cost, 3);

  // Add a second cost. This second cost shares some variables with the first
  // cost.
  const Eigen::Vector3d a2(0, 20, 30);
  const double b2{40};
  const Vector3<symbolic::Variable> var2(x_[2], x_[0], x_[1]);
  linear_costs.emplace_back(std::make_shared<LinearCost>(a2, b2), var2);
  AggregateLinearCosts(linear_costs, &linear_coeff, &vars, &constant_cost);
  EXPECT_TRUE(
      CompareMatrices(Eigen::VectorXd(linear_coeff), Eigen::Vector2d(21, 32)));
  EXPECT_EQ(vars.rows(), 2);
  EXPECT_EQ(vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(vars(1).get_id(), x_[1].get_id());
  EXPECT_EQ(constant_cost, 43);
  // Check if the symbolic expression is the same.
  EXPECT_PRED2(symbolic::test::ExprEqual,
               Eigen::VectorXd(linear_coeff).dot(vars),
               (a1.dot(var1) + a2.dot(var2)).Expand());
}

TEST_F(TestAggregateCostsAndConstraints, QuadraticAndLinearCosts) {
  std::vector<Binding<QuadraticCost>> quadratic_costs;
  std::vector<Binding<LinearCost>> linear_costs;
  // One quadratic and one linear cost.
  Eigen::Matrix3d Q1;
  // clang-format off
  Q1 << 1, 2, 0,
        2, 3, 4,
        0, 4, 5;
  // clang-format on
  const Eigen::Vector3d b1(1, 0, 2);
  const double c1{3};
  const Vector3<symbolic::Variable> vars1(x_[0], x_[2], x_[1]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q1, b1, c1),
                               vars1);

  const Eigen::Vector3d b2(2, -1, 0);
  const double c2{1};
  const Vector3<symbolic::Variable> vars2(x_[0], x_[1], x_[3]);
  linear_costs.emplace_back(std::make_shared<LinearCost>(b2, c2), vars2);

  Eigen::SparseMatrix<double> Q_lower;
  VectorX<symbolic::Variable> quadratic_vars, linear_vars;
  Eigen::SparseVector<double> linear_coeff;
  double constant_cost;
  AggregateQuadraticAndLinearCosts(quadratic_costs, linear_costs, &Q_lower,
                                   &quadratic_vars, &linear_coeff, &linear_vars,
                                   &constant_cost);
  EXPECT_TRUE(
      CompareMatrices(Eigen::MatrixXd(Q_lower),
                      Eigen::MatrixXd(Q1.triangularView<Eigen::Lower>())));
  EXPECT_EQ(quadratic_vars.rows(), 3);
  EXPECT_EQ(quadratic_vars(0).get_id(), vars1(0).get_id());
  EXPECT_EQ(quadratic_vars(1).get_id(), vars1(1).get_id());
  EXPECT_EQ(quadratic_vars(2).get_id(), vars1(2).get_id());
  EXPECT_TRUE(
      CompareMatrices(Eigen::VectorXd(linear_coeff), Eigen::Vector2d(3, 1)));
  EXPECT_EQ(linear_vars.rows(), 2);
  EXPECT_EQ(linear_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(linear_vars(1).get_id(), x_[1].get_id());
  EXPECT_EQ(constant_cost, c1 + c2);

  // More quadratic and linear costs.
  Eigen::Matrix2d Q3;
  // clang-format off
  Q3 << 10, 30,
        30, 40;
  // clang-format on
  const Eigen::Vector2d b3(20, 30);
  const double c3{40};
  const Vector2<symbolic::Variable> vars3(x_[3], x_[1]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q3, b3, c3),
                               vars3);
  const Eigen::Vector4d b4(20, 0, 30, 40);
  const double c4{10};
  const Vector4<symbolic::Variable> vars4(x_[2], x_[1], x_[3], x_[0]);
  linear_costs.emplace_back(std::make_shared<LinearCost>(b4, c4), vars4);
  AggregateQuadraticAndLinearCosts(quadratic_costs, linear_costs, &Q_lower,
                                   &quadratic_vars, &linear_coeff, &linear_vars,
                                   &constant_cost);
  Eigen::Matrix4d Q_lower_expected;
  // clang-format off
  Q_lower_expected << 1, 0, 0, 0,
                      2, 3, 0, 0,
                      0, 4, 45, 0,
                      0, 0, 30, 10;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(Q_lower),
                              Eigen::MatrixXd(Q_lower_expected)));
  EXPECT_EQ(quadratic_vars.rows(), 4);
  EXPECT_EQ(quadratic_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(quadratic_vars(1).get_id(), x_[2].get_id());
  EXPECT_EQ(quadratic_vars(2).get_id(), x_[1].get_id());
  EXPECT_EQ(quadratic_vars(3).get_id(), x_[3].get_id());

  EXPECT_TRUE(CompareMatrices(Eigen::VectorXd(linear_coeff),
                              Eigen::Vector4d(43, 31, 50, 20)));
  EXPECT_EQ(linear_vars.rows(), 4);
  EXPECT_EQ(linear_vars(0).get_id(), x_[0].get_id());
  EXPECT_EQ(linear_vars(1).get_id(), x_[1].get_id());
  EXPECT_EQ(linear_vars(2).get_id(), x_[3].get_id());
  EXPECT_EQ(linear_vars(3).get_id(), x_[2].get_id());
  EXPECT_EQ(constant_cost, c1 + c2 + c3 + c4);
}

TEST_F(TestAggregateCostsAndConstraints, AggregateBoundingBoxConstraints1) {
  // Test AggregateBoundingBoxConstraints with input being
  // std::vector<Binding<BoundingBoxConstraint>>
  std::vector<Binding<BoundingBoxConstraint>> bounding_box_constraints{};
  auto result = AggregateBoundingBoxConstraints(bounding_box_constraints);
  EXPECT_EQ(result.size(), 0);
  // 1 <= x0 <= 2
  // 3 <= x2 <= 5
  bounding_box_constraints.emplace_back(
      std::make_shared<BoundingBoxConstraint>(Eigen::Vector2d(1, 3),
                                              Eigen::Vector2d(2, 5)),
      Vector2<symbolic::Variable>(x_[0], x_[2]));
  result = AggregateBoundingBoxConstraints(bounding_box_constraints);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result.at(x_[0]).lower, 1);
  EXPECT_EQ(result.at(x_[0]).upper, 2);
  EXPECT_EQ(result.at(x_[2]).lower, 3);
  EXPECT_EQ(result.at(x_[2]).upper, 5);

  // 2 <= x2 <= 4
  // 1.5 <= x0 <= inf
  // 4 <= x1 <= inf
  bounding_box_constraints.emplace_back(
      std::make_shared<BoundingBoxConstraint>(Eigen::Vector3d(2, 1.5, 4),
                                              Eigen::Vector3d(4, kInf, kInf)),
      Vector3<symbolic::Variable>(x_[2], x_[0], x_[1]));
  result = AggregateBoundingBoxConstraints(bounding_box_constraints);
  EXPECT_EQ(result.size(), 3);
  EXPECT_EQ(result.at(x_[0]).lower, 1.5);
  EXPECT_EQ(result.at(x_[0]).upper, 2);
  EXPECT_EQ(result.at(x_[1]).lower, 4);
  EXPECT_EQ(result.at(x_[1]).upper, kInf);
  EXPECT_EQ(result.at(x_[2]).lower, 3);
  EXPECT_EQ(result.at(x_[2]).upper, 4);

  // -inf <= x1 <= 2. The returned bounds for x1 should be 4 <= x1 <= 2. This
  // test that we can return the bounds even if the lower bound is higher than
  // the upper bound.
  bounding_box_constraints.emplace_back(std::make_shared<BoundingBoxConstraint>(
      Vector1d(-kInf), Vector1d(2)), Vector1<symbolic::Variable>(x_[1]));
  result = AggregateBoundingBoxConstraints(bounding_box_constraints);
  EXPECT_EQ(result.size(), 3);
  // Only the bound of x_[1] should change, the rest should be the same.
  EXPECT_EQ(result.at(x_[0]).lower, 1.5);
  EXPECT_EQ(result.at(x_[0]).upper, 2);
  EXPECT_EQ(result.at(x_[1]).lower, 4);
  EXPECT_EQ(result.at(x_[1]).upper, 2);
  EXPECT_EQ(result.at(x_[2]).lower, 3);
  EXPECT_EQ(result.at(x_[2]).upper, 4);
}

TEST_F(TestAggregateCostsAndConstraints, AggregateBoundingBoxConstraints2) {
  // Test AggregateBoundingBoxConstraints with input being MathematicalPorgram.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  prog.AddBoundingBoxConstraint(1, 3, x.tail<2>());
  prog.AddBoundingBoxConstraint(-1, 2, x(0));
  prog.AddBoundingBoxConstraint(-kInf, 2, x(3));
  Eigen::VectorXd lower, upper;
  AggregateBoundingBoxConstraints(prog, &lower, &upper);
  EXPECT_TRUE(CompareMatrices(lower, Eigen::Vector4d(-1, -kInf, 1, 1)));
  EXPECT_TRUE(CompareMatrices(upper, Eigen::Vector4d(2, kInf, 3, 2)));
}

GTEST_TEST(TestFindNonconvexQuadraticCost, Test) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto convex_cost1 = prog.AddQuadraticCost(x(0) * x(0) + 1);
  auto convex_cost2 = prog.AddQuadraticCost(x(1) * x(1) + 2 * x(0) + 2);
  auto nonconvex_cost1 = prog.AddQuadraticCost(-x(0) * x(0) + 2 * x(1));
  auto nonconvex_cost2 = prog.AddQuadraticCost(-x(0) * x(0) + x(1) * x(1));
  EXPECT_EQ(FindNonconvexQuadraticCost({convex_cost1, convex_cost2}), nullptr);
  EXPECT_EQ(FindNonconvexQuadraticCost({convex_cost1, nonconvex_cost1})
                ->evaluator()
                .get(),
            nonconvex_cost1.evaluator().get());
  EXPECT_EQ(FindNonconvexQuadraticCost(
                {convex_cost1, nonconvex_cost1, nonconvex_cost2})
                ->evaluator()
                .get(),
            nonconvex_cost1.evaluator().get());
}

namespace internal {
GTEST_TEST(CheckConvexSolverAttributes, Test) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto convex_cost = prog.AddQuadraticCost(x(0) * x(0) + x(1));
  // Requires linear cost, but program has a quadratic cost.
  ProgramAttributes required_capabilities{ProgramAttribute::kLinearCost};
  EXPECT_FALSE(
      CheckConvexSolverAttributes(prog, required_capabilities, "foo", nullptr));
  std::string explanation;
  EXPECT_FALSE(CheckConvexSolverAttributes(prog, required_capabilities, "foo",
                                           &explanation));
  EXPECT_THAT(explanation,
              HasSubstr("foo is unable to solve because a QuadraticCost was "
                        "declared but is not supported"));

  // Test when the required capabilities matches with the program.
  required_capabilities = {ProgramAttribute::kQuadraticCost};
  EXPECT_TRUE(
      CheckConvexSolverAttributes(prog, required_capabilities, "foo", nullptr));
  EXPECT_TRUE(CheckConvexSolverAttributes(prog, required_capabilities, "foo",
                                          &explanation));
  EXPECT_TRUE(explanation.empty());

  // program has a non-convex quadratic cost.
  auto nonconvex_cost = prog.AddQuadraticCost(-x(1) * x(1));
  // Use a description that would never be mistaken as any other part of the
  // error message.
  const std::string description{"lorem ipsum"};
  nonconvex_cost.evaluator()->set_description(description);
  EXPECT_FALSE(
      CheckConvexSolverAttributes(prog, required_capabilities, "foo", nullptr));
  EXPECT_FALSE(CheckConvexSolverAttributes(prog, required_capabilities, "foo",
                                           &explanation));
  EXPECT_THAT(explanation, HasSubstr("is non-convex"));
  EXPECT_THAT(explanation, HasSubstr("foo"));
  EXPECT_THAT(explanation, HasSubstr(description));
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake

#include "drake/solvers/aggregate_costs_constraints.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

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
  bounding_box_constraints.emplace_back(
      std::make_shared<BoundingBoxConstraint>(Vector1d(-kInf), Vector1d(2)),
      Vector1<symbolic::Variable>(x_[1]));
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

  std::vector<double> lower_vec, upper_vec;
  AggregateBoundingBoxConstraints(prog, &lower_vec, &upper_vec);
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<Eigen::VectorXd>(lower_vec.data(), lower_vec.size()),
      Eigen::Vector4d(-1, -kInf, 1, 1)));
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<Eigen::VectorXd>(upper_vec.data(), upper_vec.size()),
      Eigen::Vector4d(2, kInf, 3, 2)));
}

void CheckAggregateDuplicateVariables(const Eigen::SparseMatrix<double>& A,
                                      const VectorX<symbolic::Variable>& vars) {
  Eigen::SparseMatrix<double> A_new;
  VectorX<symbolic::Variable> vars_new;
  AggregateDuplicateVariables(A, vars, &A_new, &vars_new);
  EXPECT_EQ(A.rows(), A_new.rows());
  for (int i = 0; i < A.rows(); ++i) {
    EXPECT_PRED2(symbolic::test::ExprEqual,
                 A.toDense().row(i).dot(vars).Expand(),
                 A_new.toDense().row(i).dot(vars_new).Expand());
  }
  // Make sure vars_new doesn't have duplicated variables.
  std::unordered_set<symbolic::Variable::Id> vars_new_set;
  for (int i = 0; i < vars_new.rows(); ++i) {
    EXPECT_EQ(vars_new_set.find(vars_new(i).get_id()), vars_new_set.end());
    vars_new_set.insert(vars_new(i).get_id());
  }
}
TEST_F(TestAggregateCostsAndConstraints, AggregateDuplicateVariables) {
  Eigen::SparseMatrix<double> A = Eigen::RowVector3d(1, 2, 3).sparseView();
  // No duplication.
  CheckAggregateDuplicateVariables(
      A, Vector3<symbolic::Variable>(x_[0], x_[1], x_[2]));

  // A is a row vector, vars has duplication.
  CheckAggregateDuplicateVariables(
      A, Vector3<symbolic::Variable>(x_[0], x_[1], x_[0]));
  CheckAggregateDuplicateVariables(
      A, Vector3<symbolic::Variable>(x_[0], x_[0], x_[0]));

  // A is a matrix.
  A = (Eigen::Matrix<double, 2, 3>() << 0, 1, 2, 3, 4, 0)
          .finished()
          .sparseView();
  CheckAggregateDuplicateVariables(
      A, Vector3<symbolic::Variable>(x_[1], x_[0], x_[1]));
}

GTEST_TEST(TestFindNonconvexQuadraticCost, Test) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto convex_cost1 = prog.AddQuadraticCost(x(0) * x(0) + 1);
  auto convex_cost2 = prog.AddQuadraticCost(x(1) * x(1) + 2 * x(0) + 2);
  auto nonconvex_cost1 = prog.AddQuadraticCost(-x(0) * x(0) + 2 * x(1));
  auto nonconvex_cost2 = prog.AddQuadraticCost(-x(0) * x(0) + x(1) * x(1));
  EXPECT_EQ(internal::FindNonconvexQuadraticCost({convex_cost1, convex_cost2}),
            nullptr);
  EXPECT_EQ(
      internal::FindNonconvexQuadraticCost({convex_cost1, nonconvex_cost1})
          ->evaluator()
          .get(),
      nonconvex_cost1.evaluator().get());
  EXPECT_EQ(internal::FindNonconvexQuadraticCost(
                {convex_cost1, nonconvex_cost1, nonconvex_cost2})
                ->evaluator()
                .get(),
            nonconvex_cost1.evaluator().get());
}

GTEST_TEST(TestFindNonconvexQuadraticConstraint, Test) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto convex_constraint1 =
      prog.AddQuadraticConstraint(x(0) * x(0) + 1, -kInf, 1);
  auto convex_constraint2 =
      prog.AddQuadraticConstraint(x(1) * x(1) + 2 * x(0) + 2, -kInf, 3);
  auto nonconvex_constraint1 =
      prog.AddQuadraticConstraint(-x(0) * x(0) + 2 * x(1), 0, 0);
  auto nonconvex_constraint2 =
      prog.AddQuadraticConstraint(-x(0) * x(0) + x(1) * x(1), 1, 2);
  EXPECT_EQ(internal::FindNonconvexQuadraticConstraint(
                {convex_constraint1, convex_constraint2}),
            nullptr);
  EXPECT_EQ(internal::FindNonconvexQuadraticConstraint(
                {convex_constraint1, nonconvex_constraint1})
                ->evaluator()
                .get(),
            nonconvex_constraint1.evaluator().get());
  EXPECT_EQ(
      internal::FindNonconvexQuadraticConstraint(
          {convex_constraint1, nonconvex_constraint1, nonconvex_constraint2})
          ->evaluator()
          .get(),
      nonconvex_constraint1.evaluator().get());
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

  // program has a non-convex quadratic constraint.
  prog.RemoveCost(convex_cost);
  prog.RemoveCost(nonconvex_cost);
  auto nonconvex_constraint = prog.AddQuadraticConstraint(x(1) * x(1), 1, kInf);
  // Use a description that would never be mistaken as any other part of the
  // error message.
  const std::string nonconvex_quadratic_constraint_description{"alibaba ipsum"};
  nonconvex_constraint.evaluator()->set_description(
      nonconvex_quadratic_constraint_description);
  required_capabilities = {ProgramAttribute::kQuadraticConstraint};
  EXPECT_FALSE(
      CheckConvexSolverAttributes(prog, required_capabilities, "foo", nullptr));
  EXPECT_FALSE(CheckConvexSolverAttributes(prog, required_capabilities, "foo",
                                           &explanation));
  EXPECT_THAT(explanation, HasSubstr("is non-convex"));
  EXPECT_THAT(explanation, HasSubstr("foo"));
  EXPECT_THAT(explanation,
              HasSubstr(nonconvex_quadratic_constraint_description));
}

GTEST_TEST(ParseLinearCosts, Test) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const auto y = prog.NewContinuousVariables<3>();
  prog.AddLinearCost(x[0] + 2 * y[0] + 1);
  prog.AddLinearCost(x[1] - 2 * x[0] + 3 * y[1] - y[0] + 2);
  std::vector<double> c(prog.num_vars(), 0);
  double constant = 0;
  ParseLinearCosts(prog, &c, &constant);
  // The aggregated costs are -x[0] + x[1] + y[0] + 3y[1] + 3
  Eigen::Matrix<double, 5, 1> coeff =
      (Eigen::Matrix<double, 5, 1>() << -1, 1, 1, 3, 0).finished();
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::Matrix<double, 5, 1>>(c.data()),
                              coeff));
  EXPECT_EQ(constant, 3);
  // Now start with a non-zero c and constant.
  c = {1, 2, 3, 4, 5};
  constant = 2;
  Eigen::Matrix<double, 5, 1> c_expected =
      coeff + Eigen::Map<Eigen::Matrix<double, 5, 1>>(c.data());
  double constant_expected = constant + 3;
  ParseLinearCosts(prog, &c, &constant);
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::Matrix<double, 5, 1>>(c.data()),
                              c_expected));
  EXPECT_EQ(constant, constant_expected);
}

GTEST_TEST(ParseLinearEqualityConstraints, Test) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const auto y = prog.NewContinuousVariables<2>();
  prog.AddLinearEqualityConstraint(
      Eigen::RowVector3d(1, 2, 4), 1,
      Vector3<symbolic::Variable>(x(0), x(0), y(1)));
  prog.AddLinearEqualityConstraint((Eigen::Matrix2d() << 1, 2, 3, 4).finished(),
                                   Eigen::Vector2d(2, 3),
                                   Vector2<symbolic::Variable>(x(1), y(0)));
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  std::vector<int> linear_eq_y_start_indices;
  int num_linear_equality_constraints_rows = 0;
  ParseLinearEqualityConstraints(prog, &A_triplets, &b, &A_row_count,
                                 &linear_eq_y_start_indices,
                                 &num_linear_equality_constraints_rows);
  Eigen::SparseMatrix<double> A(3, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::MatrixXd A_expected(3, prog.num_vars());
  // clang-format off
  A_expected << 3, 0, 0, 4,
                0, 1, 2, 0,
                0, 3, 4, 0;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::Vector3d>(b.data()),
                              Eigen::Vector3d(1, 2, 3)));
  EXPECT_EQ(A_row_count, 3);
  std::vector<int> linear_eq_y_start_indices_expected{0, 1};
  EXPECT_EQ(linear_eq_y_start_indices, linear_eq_y_start_indices_expected);
  EXPECT_EQ(num_linear_equality_constraints_rows, 3);

  // Use a non-zero A_row_count. The row indices should be offset by the initial
  // A_row_count.
  const int A_rows_offset = 2;
  A_row_count = A_rows_offset;
  A_triplets.clear();
  b = std::vector<double>{0, 0};
  linear_eq_y_start_indices.clear();
  ParseLinearEqualityConstraints(prog, &A_triplets, &b, &A_row_count,
                                 &linear_eq_y_start_indices,
                                 &num_linear_equality_constraints_rows);
  Eigen::SparseMatrix<double> A2(A_rows_offset + 3, prog.num_vars());
  A2.setFromTriplets(A_triplets.begin(), A_triplets.end());
  EXPECT_TRUE(CompareMatrices(A2.toDense().bottomRows(3), A_expected));
  EXPECT_TRUE(
      CompareMatrices(Eigen::Map<Eigen::Vector3d>(b.data() + A_rows_offset),
                      Eigen::Vector3d(1, 2, 3)));
  EXPECT_EQ(A_row_count, A_rows_offset + 3);
  linear_eq_y_start_indices_expected =
      std::vector<int>{A_rows_offset, A_rows_offset + 1};
  EXPECT_EQ(linear_eq_y_start_indices, linear_eq_y_start_indices_expected);
  EXPECT_EQ(num_linear_equality_constraints_rows, 3);
}

GTEST_TEST(ParseLinearConstraints, Test) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const auto y = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(
      (Eigen::Matrix3d() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished(),
      Eigen::Vector3d(-kInf, 3, 2), Eigen::Vector3d(1, kInf, 4),
      Vector3<symbolic::Variable>(x(0), y(0), x(0)));
  prog.AddLinearConstraint(Eigen::RowVector3d(1, 2, 3), -1, 3,
                           Vector3<symbolic::Variable>(y(0), y(1), x(1)));
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  std::vector<std::vector<std::pair<int, int>>> linear_constraint_dual_indices;
  int num_linear_constraint_rows;
  ParseLinearConstraints(prog, &A_triplets, &b, &A_row_count,
                         &linear_constraint_dual_indices,
                         &num_linear_constraint_rows);
  EXPECT_EQ(num_linear_constraint_rows, 6);
  Eigen::SparseMatrix<double> A(6, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::MatrixXd A_expected(6, prog.num_vars());
  // clang-format off
  A_expected << 4, 0, 2, 0,
                -10, 0, -5, 0,
                -16, 0, -8, 0,
                16, 0, 8, 0,
                0, -3, -1, -2,
                0, 3, 1, 2;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Vector6d>(b.data()),
                              (Vector6d() << 1, -3, -2, 4, 1, 3).finished()));
  EXPECT_EQ(A_row_count, 6);
  std::vector<std::vector<std::pair<int, int>>>
      linear_constraint_dual_indices_expected;
  linear_constraint_dual_indices_expected.push_back({{-1, 0}, {1, -1}, {2, 3}});
  linear_constraint_dual_indices_expected.push_back({{4, 5}});
  EXPECT_EQ(linear_constraint_dual_indices,
            linear_constraint_dual_indices_expected);
  EXPECT_EQ(num_linear_constraint_rows, 6);
}

GTEST_TEST(ParseQuadraticCosts, Test) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  prog.AddQuadraticCost((Eigen::Matrix2d() << 1, 3, 2, 4).finished(),
                        Eigen::Vector2d(1, 2), 1, x.tail<2>());
  prog.AddQuadraticCost(
      (Eigen::Matrix3d() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished(),
      Eigen::Vector3d(2, 3, 4), 2,
      Vector3<symbolic::Variable>(x(0), x(1), x(0)));
  std::vector<Eigen::Triplet<double>> P_upper_triplets;
  std::vector<double> c(prog.num_vars(), 0);
  double constant;
  ParseQuadraticCosts(prog, &P_upper_triplets, &c, &constant);
  // The total cost is 0.5 * (20*x0^2 + 6*x1^2 + 4*x2^2 + 5*x1*x2 + 20*x0*x1) +
  // 6*x0 + 4*x1 + 2*x(2) + 3
  Eigen::SparseMatrix<double> P_upper(prog.num_vars(), prog.num_vars());
  P_upper.setFromTriplets(P_upper_triplets.begin(), P_upper_triplets.end());
  Eigen::Matrix3d P_upper_expected;
  // clang-format off
  P_upper_expected << 20, 10, 0,
                       0,  6, 2.5,
                       0,  0, 4;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(P_upper.toDense(), P_upper_expected));
}

GTEST_TEST(ParseL2NormCosts, Test) {
  MathematicalProgram prog{};
  auto x = prog.NewContinuousVariables<3>();
  Eigen::Matrix<double, 3, 2> A1;
  A1 << 1, 2, 3, 4, 5, 6;
  const Eigen::Vector3d b1(-1, -2, -3);
  auto cost1 = prog.AddL2NormCost(A1, b1, x.tail<2>());

  int num_solver_variables = prog.num_vars();
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  std::vector<int> second_order_cone_length;
  std::vector<int> lorentz_cone_y_start_indices;
  std::vector<double> cost_coeffs(prog.num_vars(), 0.0);
  std::vector<int> t_slack_indices;
  // First test with a single L2NormCost.
  ParseL2NormCosts(prog, &num_solver_variables, &A_triplets, &b, &A_row_count,
                   &second_order_cone_length, &lorentz_cone_y_start_indices,
                   &cost_coeffs, &t_slack_indices);
  EXPECT_EQ(num_solver_variables, 1 + prog.num_vars());
  Eigen::SparseMatrix<double> A(1 + A1.rows(), prog.num_vars() + 1);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::MatrixXd A_expected(1 + A1.rows(), prog.num_vars() + 1);
  A_expected.setZero();
  A_expected(0, prog.num_vars()) = -1;
  A_expected.block(1, 1, A1.rows(), A1.cols()) = -A1;
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  Eigen::VectorXd b_expected(1 + b1.rows());
  b_expected.setZero();
  b_expected.tail(b1.rows()) = b1;
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::VectorXd>(b.data(), b.size()),
                              b_expected));
  EXPECT_EQ(A_row_count, 1 + A1.rows());
  EXPECT_EQ(second_order_cone_length.size(), 1);
  EXPECT_EQ(second_order_cone_length[0], A1.rows() + 1);
  EXPECT_EQ(lorentz_cone_y_start_indices.size(), 1);
  EXPECT_EQ(lorentz_cone_y_start_indices[0], 0);
  Eigen::VectorXd cost_coeffs_expected(prog.num_vars() + 1);
  cost_coeffs_expected.setZero();
  cost_coeffs_expected(prog.num_vars()) = 1;
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<Eigen::VectorXd>(cost_coeffs.data(), cost_coeffs.size()),
      cost_coeffs_expected));
  EXPECT_EQ(t_slack_indices.size(), 1);
  EXPECT_EQ(t_slack_indices[0], prog.num_vars());

  // Test with multiple L2NormCost.
  num_solver_variables = prog.num_vars();
  A_row_count = 0;
  A_triplets.clear();
  b.clear();
  second_order_cone_length.clear();
  lorentz_cone_y_start_indices.clear();
  cost_coeffs = std::vector<double>(prog.num_vars(), 0);
  t_slack_indices.clear();
  Eigen::Matrix<double, 2, 2> A2;
  A2 << -1, -3, -5, -7;
  const Eigen::Vector2d b2(5, 10);
  auto cost2 = prog.AddL2NormCost(A2, b2, x.head<2>());
  ParseL2NormCosts(prog, &num_solver_variables, &A_triplets, &b, &A_row_count,
                   &second_order_cone_length, &lorentz_cone_y_start_indices,
                   &cost_coeffs, &t_slack_indices);
  EXPECT_EQ(num_solver_variables, prog.num_vars() + 2);
  A_expected.resize(2 + A1.rows() + A2.rows(), prog.num_vars() + 2);
  A_expected.setZero();
  A_expected(0, prog.num_vars()) = -1;
  A_expected.block(1, 1, A1.rows(), 2) = -A1;
  A_expected(A1.rows() + 1, prog.num_vars() + 1) = -1;
  A_expected.block(2 + A1.rows(), 0, A2.rows(), 2) = -A2;
  A.resize(2 + A1.rows() + A2.rows(), 2 + prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  b_expected.resize(A1.rows() + A2.rows() + 2);
  b_expected.setZero();
  b_expected.segment(1, A1.rows()) = b1;
  b_expected.segment(2 + A1.rows(), A2.rows()) = b2;
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::VectorXd>(b.data(), b.size()),
                              b_expected));
  EXPECT_EQ(A_row_count, 2 + A1.rows() + A2.rows());
  EXPECT_EQ(second_order_cone_length.size(), 2);
  EXPECT_EQ(second_order_cone_length[0], A1.rows() + 1);
  EXPECT_EQ(second_order_cone_length[1], A2.rows() + 1);
  EXPECT_EQ(lorentz_cone_y_start_indices.size(), 2);
  EXPECT_EQ(lorentz_cone_y_start_indices[0], 0);
  EXPECT_EQ(lorentz_cone_y_start_indices[1], 1 + A1.rows());
  cost_coeffs_expected.resize(prog.num_vars() + 2);
  cost_coeffs_expected.setZero();
  cost_coeffs_expected(prog.num_vars()) = 1;
  cost_coeffs_expected(prog.num_vars() + 1) = 1;
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<Eigen::VectorXd>(cost_coeffs.data(), cost_coeffs.size()),
      cost_coeffs_expected));
  EXPECT_EQ(t_slack_indices.size(), 2);
  EXPECT_EQ(t_slack_indices[0], prog.num_vars());
  EXPECT_EQ(t_slack_indices[1], prog.num_vars() + 1);
}

GTEST_TEST(ParseSecondOrderConeConstraints, LorentzCone) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();

  prog.AddLorentzConeConstraint(
      (Eigen::Matrix<double, 4, 3>() << 1, 0, 2, -1, 2, 3, 1, 2, 4, -2, 1, 2)
          .finished(),
      Eigen::Vector4d(1, 2, 3, 4),
      Vector3<symbolic::Variable>(x(1), x(0), x(1)));

  prog.AddLorentzConeConstraint(
      (Eigen::Matrix<double, 4, 2>() << 1, 2, 3, 4, 5, 6, 7, 8).finished(),
      Eigen::Vector4d(-1, -2, -3, -4), Vector2<symbolic::Variable>(x(2), x(0)));
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count{0};
  std::vector<int> second_order_cone_length;
  std::vector<int> lorentz_cone_y_start_indices;
  std::vector<int> rotated_lorentz_cone_y_start_indices;
  ParseSecondOrderConeConstraints(
      prog, &A_triplets, &b, &A_row_count, &second_order_cone_length,
      &lorentz_cone_y_start_indices, &rotated_lorentz_cone_y_start_indices);
  EXPECT_EQ(A_row_count, 8);
  Eigen::SparseMatrix<double> A(8, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::MatrixXd A_expected(8, prog.num_vars());
  // clang-format off
  A_expected << 0, -3, 0,
                -2, -2, 0,
                -2, -5, 0,
                -1, 0, 0,
                 -2, 0, -1,
                 -4, 0, -3,
                 -6, 0, -5,
                 -8, 0, -7;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  Eigen::Matrix<double, 8, 1> b_expected;
  b_expected << 1, 2, 3, 4, -1, -2, -3, -4;
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::Matrix<double, 8, 1>>(b.data()),
                              b_expected));
  EXPECT_EQ(second_order_cone_length, std::vector<int>({4, 4}));
  EXPECT_EQ(lorentz_cone_y_start_indices, std::vector<int>({0, 4}));
  EXPECT_EQ(rotated_lorentz_cone_y_start_indices, std::vector<int>());
}

GTEST_TEST(ParseSecondOrderConeConstraints, RotatedLorentzConeConstraint) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  prog.AddRotatedLorentzConeConstraint(
      (Eigen::Matrix<double, 4, 3>() << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)
          .finished(),
      Eigen::Vector4d(0, 1, 2, 3),
      Vector3<symbolic::Variable>(x(1), x(0), x(1)));
  prog.AddRotatedLorentzConeConstraint(
      (Eigen::Matrix<double, 3, 4>() << -1, -2, -3, -4, -5, -6, -7, -8, -9, -10,
       -11, -12)
          .finished(),
      Eigen::Vector3d(5, 6, 7),
      Vector4<symbolic::Variable>(x(1), x(0), x(0), x(2)));
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count{0};
  std::vector<int> second_order_cone_length;
  std::vector<int> lorentz_cone_y_start_indices;
  std::vector<int> rotated_lorentz_cone_y_start_indices;
  ParseSecondOrderConeConstraints(
      prog, &A_triplets, &b, &A_row_count, &second_order_cone_length,
      &lorentz_cone_y_start_indices, &rotated_lorentz_cone_y_start_indices);
  EXPECT_EQ(A_row_count, 7);
  Eigen::SparseMatrix<double> A(7, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::MatrixXd A_expected(7, prog.num_vars());
  // clang-format off
  A_expected << -3.5, -7, 0,
                 1.5, 3, 0,
                 -8, -16, 0,
                 -11, -22, 0,
                 9, 3, 6,
                 -4, -2, -2,
                 21, 9, 12;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  Eigen::Matrix<double, 7, 1> b_expected;
  b_expected << 0.5, -0.5, 2, 3, 5.5, -0.5, 7;
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::Matrix<double, 7, 1>>(b.data()),
                              b_expected));
  EXPECT_EQ(second_order_cone_length, std::vector<int>({4, 3}));
  EXPECT_EQ(lorentz_cone_y_start_indices, std::vector<int>());
  EXPECT_EQ(rotated_lorentz_cone_y_start_indices, std::vector<int>({0, 4}));
}

GTEST_TEST(ParseExponentialConeConstraints, Test) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  prog.AddExponentialConeConstraint(
      (Eigen::Matrix<double, 3, 4>() << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)
          .finished()
          .sparseView(),
      Eigen::Vector3d(1, 2, 3),
      Vector4<symbolic::Variable>(x(1), x(0), x(2), x(0)));
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  ParseExponentialConeConstraints(prog, &A_triplets, &b, &A_row_count);
  EXPECT_EQ(A_row_count, 3);
  Eigen::SparseMatrix<double> A(3, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::Matrix3d A_expected;
  // clang-format off
  A_expected << -22, -9, -11,
                -14, -5, -7,
                -6, -1, -3;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  EXPECT_EQ(b.size(), 3);
  EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::Vector3d>(b.data()),
                              Eigen::Vector3d(3, 2, 1)));
}

GTEST_TEST(ParsePositiveSemidefiniteConstraints, TestPsd) {
  // Test parsing prog.positive_semidefinite_constraints.
  MathematicalProgram prog;
  const auto X = prog.NewSymmetricContinuousVariables<3>();
  prog.AddPositiveSemidefiniteConstraint(X);
  const auto Y = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(Y);

  auto check_psd = [&prog](bool upper_triangular) {
    SCOPED_TRACE(fmt::format("upper_triangular = {}", upper_triangular));
    std::vector<Eigen::Triplet<double>> A_triplets;
    // Assume that A*x+s = b already contain `A_row_count_old` number of
    // constraints. We check if the new constraints are appended to A*x+s = b.
    int A_row_count_old = 2;
    std::vector<double> b(A_row_count_old);
    int A_row_count = A_row_count_old;
    std::vector<std::optional<int>> psd_cone_length;
    std::vector<std::optional<int>> lmi_cone_length;
    std::vector<std::optional<int>> psd_y_start_indices;
    std::vector<std::optional<int>> lmi_y_start_indices;
    ParsePositiveSemidefiniteConstraints(
        prog, upper_triangular, &A_triplets, &b, &A_row_count, &psd_cone_length,
        &lmi_cone_length, &psd_y_start_indices, &lmi_y_start_indices);
    EXPECT_EQ(psd_cone_length,
              std::vector<std::optional<int>>({{3}, {std::nullopt}}));
    EXPECT_TRUE(lmi_cone_length.empty());
    EXPECT_EQ(psd_y_start_indices, std::vector<std::optional<int>>(
                                       {{A_row_count_old}, {std::nullopt}}));
    EXPECT_TRUE(lmi_y_start_indices.empty());
    // We add 3 * (3+1) / 2 = 6 rows in A for "X is psd", and 0
    // rows in A for "Y is psd" since Y is a 2x2 matrix, which should be imposed
    // as a second order cone constraint.
    EXPECT_EQ(A_row_count, A_row_count_old + 3 * (3 + 1) / 2);
    const double sqrt2 = std::sqrt(2);
    Eigen::SparseMatrix<double> A(A_row_count, prog.num_vars());
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    Eigen::MatrixXd A_expected(A_row_count_old + 6, prog.num_vars());
    A_expected.setZero();
    // [1 √2 √2]
    // [√2 1 √2]
    // [√2 √2 1]
    // and
    // [1 √2]
    // [√2 1]
    if (upper_triangular) {
      // If we use the upper triangular part of the symmetric matrix, to impose
      // the constraint that 3 by 3 matrix X is psd, we need the constraint
      //   -X(0, 0) + s(0) = 0
      // -√2X(0, 1) + s(1) = 0
      //   -X(1, 1) + s(2) = 0
      // -√2X(0, 2) + s(3) = 0
      // -√2X(1, 2) + s(4) = 0
      //   -X(2, 2) + s(5) = 0
      // and
      // [s(0) s(1) s(3)]
      // [s(1) s(2) s(4)]  is psd.
      // [s(3) s(4) s(5)]
      // Below we write
      // A_expected(A_row_count_old + s_index, X_index) = -X_coeff.
      A_expected(A_row_count_old + 0, 0 /* X(0, 0) */) = -1;
      A_expected(A_row_count_old + 1, 1 /* X(0, 1) */) = -sqrt2;
      A_expected(A_row_count_old + 2, 3 /* X(1, 1) */) = -1;
      A_expected(A_row_count_old + 3, 2 /* X(0, 2) */) = -sqrt2;
      A_expected(A_row_count_old + 4, 4 /* X(1, 2) */) = -sqrt2;
      A_expected(A_row_count_old + 5, 5 /* X(2, 2) */) = -1;
    } else {
      // If we use the lower triangular part of the symmetric matrix, to impose
      // the constraint that 3 by 3 matrix X is psd, we need the constraint
      //   -X(0, 0) + s(0) = 0
      // -√2X(1, 0) + s(1) = 0
      // -√2X(2, 0) + s(2) = 0
      //   -X(1, 1) + s(3) = 0
      // -√2X(2, 1) + s(4) = 0
      //   -X(2, 2) + s(5) = 0
      // and
      // [s(0) s(1) s(2)]
      // [s(1) s(3) s(4)]  is psd.
      // [s(2) s(4) s(5)]
      // Below we write
      // A_expected(A_row_count_old + s_index, X_index) = -X_coeff.
      A_expected(A_row_count_old + 0, 0) = -1;
      A_expected(A_row_count_old + 1, 1) = -sqrt2;
      A_expected(A_row_count_old + 2, 2) = -sqrt2;
      A_expected(A_row_count_old + 3, 3) = -1;
      A_expected(A_row_count_old + 4, 4) = -sqrt2;
      A_expected(A_row_count_old + 5, 5) = -1;
    }
    EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
    EXPECT_EQ(b.size(), A_row_count);
    for (int i = A_row_count_old; i < A_row_count; ++i) {
      EXPECT_EQ(b[i], 0);
    }
  };
  check_psd(true);
  check_psd(false);
}

GTEST_TEST(ParsePositiveSemidefiniteConstraints, TestLmi) {
  // Test parsing prog.linear_matrix_inequality_constraints.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  auto symmetrize_matrix = [](const Eigen::Ref<const Eigen::MatrixXd>& A) {
    return (A + A.transpose()) / 2;
  };

  const auto lmi_constraint = prog.AddLinearMatrixInequalityConstraint(
      {symmetrize_matrix(
           (Eigen::Matrix3d() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished()),
       symmetrize_matrix(
           (Eigen::Matrix3d() << 1, 3, 5, 7, 9, 2, 4, 6, 8).finished()),
       symmetrize_matrix(
           (Eigen::Matrix3d() << -1, -2, 3, 4, 5, -6, -7, -8, 9).finished()),
       Eigen::Matrix3d::Identity()},
      x);

  auto check_lmi = [&prog, &lmi_constraint](bool upper_triangular) {
    SCOPED_TRACE(fmt::format("upper_triangular = {}", upper_triangular));
    std::vector<Eigen::Triplet<double>> A_triplets;
    // Assume A*x+s=b already contains `A_row_count_old` rows. We check if the
    // new PSD constraints are appended to the existing A*x+s=b.
    const int A_row_count_old = 2;
    std::vector<double> b(A_row_count_old, 0);
    int A_row_count = A_row_count_old;
    std::vector<std::optional<int>> psd_cone_length;
    std::vector<std::optional<int>> lmi_cone_length;
    std::vector<std::optional<int>> psd_y_start_indices;
    std::vector<std::optional<int>> lmi_y_start_indices;
    ParsePositiveSemidefiniteConstraints(
        prog, upper_triangular, &A_triplets, &b, &A_row_count, &psd_cone_length,
        &lmi_cone_length, &psd_y_start_indices, &lmi_y_start_indices);
    EXPECT_EQ(A_row_count, A_row_count_old + 3 * (3 + 1) / 2);
    EXPECT_TRUE(psd_cone_length.empty());
    EXPECT_EQ(lmi_cone_length, std::vector<std::optional<int>>({{3}}));
    EXPECT_TRUE(psd_y_start_indices.empty());
    EXPECT_EQ(lmi_y_start_indices,
              std::vector<std::optional<int>>({{A_row_count_old}}));
    Eigen::SparseMatrix<double> A(A_row_count_old + 6, prog.num_vars());
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    Eigen::MatrixXd A_expected(A_row_count_old + 6, prog.num_vars());
    A_expected.setZero();
    const std::vector<Eigen::MatrixXd> F = lmi_constraint.evaluator()->F();
    const double sqrt2 = std::sqrt(2);
    if (upper_triangular) {
      // clang-format off
      A_expected.bottomRows<6>() << -F[1](0, 0), -F[2](0, 0), -F[3](0, 0),
                  -sqrt2 * F[1](0, 1), -sqrt2 * F[2](1, 0), -sqrt2 * F[3](1, 0),
                  -F[1](1, 1), -F[2](1, 1), -F[3](1, 1),
                  -sqrt2*F[1](0, 2), -sqrt2*F[2](0, 2), -sqrt2*F[3](0, 2),
                  -sqrt2 * F[1](1, 2), -sqrt2 * F[2](1, 2), -sqrt2 * F[3](1, 2),
                  -F[1](2, 2), -F[2](2, 2), -F[3](2, 2);
      // clang-format on
    } else {
      // clang-format off
      A_expected.bottomRows<6>() << -F[1](0, 0), -F[2](0, 0), -F[3](0, 0),
                -sqrt2 * F[1](1, 0), -sqrt2 * F[2](1, 0), -sqrt2 * F[3](1, 0),
                -sqrt2 * F[1](2, 0), -sqrt2 * F[2](2, 0), -sqrt2 * F[3](2, 0),
                -F[1](1, 1), -F[2](1, 1), -F[3](1, 1),
                -sqrt2 * F[1](2, 1), -sqrt2 * F[2](2, 1), -sqrt2 * F[3](2, 1),
                -F[1](2, 2), -F[2](2, 2), -F[3](2, 2);
      // clang-format on
    }
    Eigen::VectorXd b_expected(A_row_count);
    if (upper_triangular) {
      b_expected << 0, 0, F[0](0, 0), sqrt2 * F[0](0, 1), F[0](1, 1),
          sqrt2 * F[0](0, 2), sqrt2 * F[0](1, 2), F[0](2, 2);
    } else {
      b_expected << 0, 0, F[0](0, 0), sqrt2 * F[0](1, 0), sqrt2 * F[0](2, 0),
          F[0](1, 1), sqrt2 * F[0](2, 1), F[0](2, 2);
    }
    EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected, 1E-12));
    EXPECT_EQ(b.size(), A_row_count);
    EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::VectorXd>(b.data(), b.size()),
                                b_expected, 1E-12));
  };
  check_lmi(true);
  check_lmi(false);
}

GTEST_TEST(ParseScalarPositiveSemidefiniteConstraints, Test) {
  // The program contains both scalar PSD/LMI constraints and non-scalar PSD/LMI
  // constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddPositiveSemidefiniteConstraint(x);
  auto X = prog.NewSymmetricContinuousVariables<3>();
  prog.AddPositiveSemidefiniteConstraint(X);
  auto y = prog.NewContinuousVariables<2>();
  prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Ones(),
       Eigen::Matrix3d::Identity()},
      y);
  auto scalar_lmi_con = prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix<double, 1, 1>(1), Eigen::Matrix<double, 1, 1>(2),
       Eigen::Matrix<double, 1, 1>(-3)},
      y);
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  int new_positive_cone_length{};
  std::vector<std::optional<int>> scalar_psd_dual_indices;
  std::vector<std::optional<int>> scalar_lmi_dual_indices;
  ParseScalarPositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &new_positive_cone_length,
      &scalar_psd_dual_indices, &scalar_lmi_dual_indices);
  EXPECT_EQ(A_triplets.size(), 3);
  Eigen::SparseMatrix<double> A(2, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::MatrixXd A_expected(2, prog.num_vars());
  A_expected.setZero();
  A_expected(0, prog.FindDecisionVariableIndex(x(0))) = -1;
  for (int i = 0; i < y.rows(); ++i) {
    A_expected(1, prog.FindDecisionVariableIndex(y(i))) =
        -scalar_lmi_con.evaluator()->F()[i + 1](0, 0);
  }
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  const Eigen::Vector2d b_expected(0, scalar_lmi_con.evaluator()->F()[0](0, 0));
  EXPECT_TRUE(
      CompareMatrices(Eigen::Map<Eigen::Vector2d>(b.data()), b_expected));
  EXPECT_EQ(A_row_count, 2);
  EXPECT_EQ(new_positive_cone_length, 2);
  EXPECT_FALSE(scalar_psd_dual_indices[1].has_value());
  EXPECT_FALSE(scalar_lmi_dual_indices[0].has_value());

  std::vector<std::optional<int>> psd_cone_length;
  std::vector<std::optional<int>> lmi_cone_length;
  std::vector<std::optional<int>> psd_y_start_indices;
  std::vector<std::optional<int>> lmi_y_start_indices;
  ParsePositiveSemidefiniteConstraints(
      prog, /*upper_triangular=*/true, &A_triplets, &b, &A_row_count,
      &psd_cone_length, &lmi_cone_length, &psd_y_start_indices,
      &lmi_y_start_indices);
  EXPECT_EQ(psd_cone_length,
            std::vector<std::optional<int>>({{std::nullopt}, {3}}));
  EXPECT_EQ(lmi_cone_length,
            std::vector<std::optional<int>>({{3}, {std::nullopt}}));
  EXPECT_FALSE(psd_y_start_indices[0].has_value());
  EXPECT_FALSE(lmi_y_start_indices[1].has_value());
}

GTEST_TEST(Parse2x2PositiveSemidefiniteConstraints, TestPSD) {
  // A PositiveSemidefiniteConstraint on a 2 x 2 matrix.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // X has repeated entries.
  Matrix2<symbolic::Variable> X;
  // clang-format off
  X << x(0), x(1),
       x(1), x(0);
  // clang-format on
  auto psd_con = prog.AddPositiveSemidefiniteConstraint(X);
  int A_row_count = 0;
  std::vector<Eigen::Triplet<double>> A_triplets{};
  std::vector<double> b{};
  int num_new_second_order_cones{0};
  std::vector<std::optional<int>> twobytwo_psd_dual_start_indices;
  std::vector<std::optional<int>> twobytwo_lmi_dual_start_indices;
  Parse2x2PositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &num_new_second_order_cones,
      &twobytwo_psd_dual_start_indices, &twobytwo_lmi_dual_start_indices);
  EXPECT_EQ(A_row_count, 3);
  EXPECT_EQ(num_new_second_order_cones, 1);
  EXPECT_EQ(b, std::vector<double>({0, 0, 0}));
  Eigen::SparseMatrix<double> A(A_row_count, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::Matrix<double, 3, 2> A_expected;
  // clang-format off
  A_expected << -2, 0,
                0, 0,
                0, -2;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A.toDense(), A_expected));
  EXPECT_EQ(twobytwo_psd_dual_start_indices,
            std::vector<std::optional<int>>({{0}}));
  EXPECT_TRUE(twobytwo_lmi_dual_start_indices.empty());
}

GTEST_TEST(Parse2x2PositiveSemidefiniteConstraints, TestLMI) {
  // A LinearMatrixInequalityConstraint on a 2x2 matrix.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  std::vector<Eigen::MatrixXd> F;
  F.push_back((Eigen::Matrix2d() << 1, 2, 2, 4).finished());
  F.push_back((Eigen::Matrix2d() << 0, 1, 1, -2).finished());
  F.push_back((Eigen::Matrix2d() << 1, 0, 0, -1).finished());
  F.push_back((Eigen::Matrix2d() << 2, 1, 1, 1).finished());
  auto lmi_con = prog.AddLinearMatrixInequalityConstraint(
      F, Vector3<symbolic::Variable>(x(0), x(1), x(0)));
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  int num_new_second_order_cones{};
  std::vector<std::optional<int>> twobytwo_psd_dual_start_indices;
  std::vector<std::optional<int>> twobytwo_lmi_dual_start_indices;
  Parse2x2PositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &num_new_second_order_cones,
      &twobytwo_psd_dual_start_indices, &twobytwo_lmi_dual_start_indices);
  EXPECT_EQ(A_row_count, 3);
  EXPECT_EQ(num_new_second_order_cones, 1);
  Eigen::SparseMatrix<double> A(3, prog.num_vars());
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  // Since A*x + s = b, we have s = -A*x+b
  VectorX<symbolic::Expression> s =
      -A.toDense() * prog.decision_variables() +
      Eigen::Map<Eigen::VectorXd>(b.data(), b.size());
  // Now evaluate F[0] + ∑ᵢF[1+i] * x(i)
  Matrix2<symbolic::Expression> lmi_expected = F[0];
  for (int i = 0; i < lmi_con.variables().rows(); ++i) {
    lmi_expected += lmi_con.evaluator()->F()[1 + i] * lmi_con.variables()(i);
  }
  EXPECT_PRED2(symbolic::test::ExprEqual,
               (lmi_expected(0, 0) + lmi_expected(1, 1)).Expand(),
               s(0).Expand());
  EXPECT_PRED2(symbolic::test::ExprEqual,
               (lmi_expected(0, 0) - lmi_expected(1, 1)).Expand(),
               s(1).Expand());
  EXPECT_PRED2(symbolic::test::ExprEqual, (2 * lmi_expected(0, 1)).Expand(),
               s(2).Expand());
  EXPECT_TRUE(twobytwo_psd_dual_start_indices.empty());
  EXPECT_EQ(twobytwo_lmi_dual_start_indices,
            std::vector<std::optional<int>>({{0}}));
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake

#include "drake/solvers/aggregate_costs_constraints.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace solvers {
namespace internal {
GTEST_TEST(AggregateQuadraticCosts, Test) {
  std::vector<symbolic::Variable> x(4);
  for (int i = 0; i < 4; ++i) {
    x[i] = symbolic::Variable(fmt::format("x{}", i));
  }

  // Test a single cost.
  std::vector<Binding<QuadraticCost>> quadratic_costs;
  Eigen::Matrix2d Q1;
  // clang-format off
  Q1 << 1, 2,
        2, 3;
  // clang-format on
  const Eigen::Vector2d b1(5, 0.);
  const double c1{2};
  const Vector2<symbolic::Variable> vars1(x[0], x[2]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q1, b1, c1),
                               vars1);

  Eigen::SparseMatrix<double> Q_lower;
  Eigen::SparseVector<double> linear_coeff;
  VectorX<symbolic::Variable> vars;
  double constant_cost;
  AggregateQuadraticCosts(quadratic_costs, &Q_lower, &linear_coeff, &vars,
                          &constant_cost);
  EXPECT_EQ(vars.rows(), 2);
  EXPECT_EQ(vars(0).get_id(), x[0].get_id());
  EXPECT_EQ(vars(1).get_id(), x[2].get_id());
  EXPECT_TRUE(
      CompareMatrices(Eigen::MatrixXd(Q_lower),
                      Eigen::MatrixXd(Q1.triangularView<Eigen::Lower>())));
  EXPECT_TRUE(CompareMatrices(Eigen::VectorXd(linear_coeff), b1));
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
  const Vector3<symbolic::Variable> vars2(x[2], x[1], x[3]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q2, b2, c2),
                               vars2);
  AggregateQuadraticCosts(quadratic_costs, &Q_lower, &linear_coeff, &vars,
                          &constant_cost);
  EXPECT_EQ(vars.rows(), 4);
  EXPECT_EQ(vars(0).get_id(), x[0].get_id());
  EXPECT_EQ(vars(1).get_id(), x[2].get_id());
  EXPECT_EQ(vars(2).get_id(), x[1].get_id());
  EXPECT_EQ(vars(3).get_id(), x[3].get_id());
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
                              Eigen::Vector4d(5, 10, 0, 50)));
  EXPECT_EQ(constant_cost, c1 + c2);

  // Add another quadratic cost.
  Eigen::Matrix2d Q3;
  Q3 << 0, 0, 0, 1;
  const Eigen::Vector2d b3(0.1, 0.5);
  const double c3 = 0.5;
  const Vector2<symbolic::Variable> vars3(x[2], x[1]);
  quadratic_costs.emplace_back(std::make_shared<QuadraticCost>(Q3, b3, c3),
                               vars3);
  AggregateQuadraticCosts(quadratic_costs, &Q_lower, &linear_coeff, &vars,
                          &constant_cost);
  // Check if the aggregated cost (in symbolic form) is correct.
  EXPECT_PRED2(
      symbolic::test::ExprEqual,
      vars.dot(Eigen::MatrixXd(
                   Eigen::MatrixXd(Q_lower).selfadjointView<Eigen::Lower>()) *
               vars)
          .Expand(),
      (vars1.dot(Q1 * vars1) + vars2.dot(Q2 * vars2) + vars3.dot(Q3 * vars3))
          .Expand());

  EXPECT_PRED2(symbolic::test::ExprEqual,
               Eigen::VectorXd(linear_coeff).dot(vars),
               (b1.dot(vars1) + b2.dot(vars2) + b3.dot(vars3)).Expand());
  EXPECT_EQ(constant_cost, c1 + c2 + c3);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake

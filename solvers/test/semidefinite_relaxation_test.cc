#include "drake/solvers/semidefinite_relaxation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace internal {

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using symbolic::Variable;

void SetRelaxationInitialGuess(const Eigen::Ref<const VectorXd>& y_expected,
                               MathematicalProgram* relaxation) {
  const int N = y_expected.size() + 1;
  MatrixX<Variable> X = Eigen::Map<const MatrixX<Variable>>(
      relaxation->positive_semidefinite_constraints()[0].variables().data(), N,
      N);
  VectorXd x_expected(N);
  x_expected << y_expected, 1;
  const MatrixXd X_expected = x_expected * x_expected.transpose();
  relaxation->SetInitialGuess(X, X_expected);
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, NoCostsNorConstraints) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  const auto relaxation = MakeSemidefiniteRelaxation(prog);

  // X is 3x3 symmetric.
  EXPECT_EQ(relaxation->num_vars(), 6);
  // X ≽ 0.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  // X(-1,-1) = 1.
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, UnsupportedCost) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  prog.AddCost(sin(y[0]));
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog),
      ".*GenericCost was declared but is not supported.");
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, UnsupportedConstraint) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  prog.AddConstraint(sin(y[0]) >= 0.2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog),
      ".*GenericConstraint was declared but is not supported.");
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, LinearCost) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  const Vector2d a(0.5, 0.7);
  const double b = 1.3;
  prog.AddLinearCost(a, b, y);
  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      a.transpose() * y_test + b, 1e-12);

  // Confirm that the decision variables of prog are also decision variables of
  // the relaxation.
  std::vector<int> indices = relaxation->FindDecisionVariableIndices(y);
  EXPECT_EQ(indices.size(), 2);
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, QuadraticCost) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  const Vector2d yd(0.5, 0.7);
  prog.AddQuadraticErrorCost(Matrix2d::Identity(), yd, y);
  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);

  SetRelaxationInitialGuess(yd, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      0, 1e-12);
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, LinearConstraint) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  MatrixXd A(3, 2);
  A << 0.5, 0.7, -0.2, 0.4, -2.3, -4.5;
  const Vector3d lb(1.3, -0.24, 0.25);
  const Vector3d ub(5.6, 0.1, 1.4);
  prog.AddLinearConstraint(A, lb, ub, y);
  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_constraints().size(), 3);

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());

  // First linear constraint is lb ≤ Ay ≤ ub.
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->GetDenseA(),
      relaxation->linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->lower_bound(),
      relaxation->linear_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->upper_bound(),
      relaxation->linear_constraints()[0].evaluator()->upper_bound()));

  // Second linear constraint is (Ay - lb)*(Ay - lb)' >= -lb*lb'.
  VectorXd value = relaxation->EvalBindingAtInitialGuess(
                                  relaxation->linear_constraints()[1]);
  MatrixXd expected =
      (A * y_test - lb) * (A * y_test - lb).transpose() - lb * lb.transpose();
  EXPECT_TRUE(CompareMatrices(Eigen::Map<MatrixXd>(value.data(), 3, 3),
                              expected, 1e-12));
  value = relaxation->linear_constraints()[1].evaluator()->lower_bound();
  expected = -lb * lb.transpose();
  EXPECT_TRUE(CompareMatrices(Eigen::Map<MatrixXd>(value.data(), 3, 3),
                              expected, 1e-12));

  // Third linear constraint is (Ay - ub)*(Ay - ub)' >= -ub*ub'.
  value = relaxation->EvalBindingAtInitialGuess(
                                  relaxation->linear_constraints()[2]);
  expected =
      (A * y_test - ub) * (A * y_test - ub).transpose() - ub * ub.transpose();
  EXPECT_TRUE(CompareMatrices(Eigen::Map<MatrixXd>(value.data(), 3, 3),
                              expected, 1e-12));
  value = relaxation->linear_constraints()[2].evaluator()->lower_bound();
  expected = -ub * ub.transpose();
  EXPECT_TRUE(CompareMatrices(Eigen::Map<MatrixXd>(value.data(), 3, 3),
                              expected, 1e-12));
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, LinearEqualityConstraint) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  MatrixXd A(3, 2);
  A << 0.5, 0.7, -0.2, 0.4, -2.3, -4.5;
  const Vector3d b(1.3, -0.24, 0.25);
  prog.AddLinearEqualityConstraint(A, b, y);
  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());

  // First linear constraint is Ay = b.
  EXPECT_TRUE(CompareMatrices(
      prog.linear_equality_constraints()[0].evaluator()->GetDenseA(),
      relaxation->linear_equality_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_equality_constraints()[0].evaluator()->lower_bound(),
      relaxation->linear_equality_constraints()[0].evaluator()->lower_bound()));

  // Second linear constraint is (Ay - lb)*(Ay - lb)' = -lb*lb'.
  VectorXd value = relaxation->EvalBindingAtInitialGuess(
                                  relaxation->linear_equality_constraints()[1]);
  MatrixXd expected =
      (A * y_test - b) * (A * y_test - b).transpose() - b * b.transpose();
  // Value is only lower-triangular portion.
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(CompareMatrices(value.segment(count, 3 - i),
                                expected.block(i, i, 3 - i, 1), 1e-12));
    count += 3 - i;
  }
  value =
      relaxation->linear_equality_constraints()[1].evaluator()->lower_bound();
  expected = -b * b.transpose();
  count = 0;
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(CompareMatrices(value.segment(count, 3 - i),
                                expected.block(i, i, 3 - i, 1), 1e-12));
    count += 3 - i;
  }
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, QuadraticConstraint) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  Matrix2d Q;
  Q << 1, 2, 3, 4;
  const Vector2d b(0.2, 0.4);
  const double lb = -.4, ub = 0.5;
  prog.AddQuadraticConstraint(Q, b, lb, ub, y);
  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_constraints().size(), 1);

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[0])[0],
      (0.5 * y_test.transpose() * Q * y_test + b.transpose() * y_test)[0],
      1e-12);

  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->lower_bound()[0],
            lb);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->upper_bound()[0],
            ub);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake

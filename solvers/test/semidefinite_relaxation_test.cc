#include "drake/solvers/semidefinite_relaxation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/matrix_util.h"

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
using symbolic::Variables;

namespace {
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

const double kInf = std::numeric_limits<double>::infinity();

}  // namespace

GTEST_TEST(MakeSemidefiniteRelaxationTest, NoCostsNorConstraints) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  const auto relaxation = MakeSemidefiniteRelaxation(prog);

  // X is 3x3 symmetric.
  EXPECT_EQ(relaxation->num_vars(), 6);
  // X ≽ 0.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  // X(-1,-1) = 1.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);

  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
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
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
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
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);

  SetRelaxationInitialGuess(yd, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      0, 1e-12);
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, BoundingBoxConstraint) {
  MathematicalProgram prog;
  const int N_VARS = 2;
  const auto y = prog.NewContinuousVariables<2>("y");

  VectorXd lb(N_VARS);
  lb << -1.5, -2.0;

  VectorXd ub(N_VARS);
  ub << kInf, 2.3;

  prog.AddBoundingBoxConstraint(lb, ub, y);

  auto relaxation = MakeSemidefiniteRelaxation(prog);

  // We have 1 bounding box constraint.
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  // We have 1 linear constraint due to the product of the bounding box
  // constraints.
  EXPECT_EQ(relaxation->linear_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 4);

  auto bbox_evaluator = relaxation->bounding_box_constraints()[0].evaluator();

  EXPECT_TRUE(CompareMatrices(lb, bbox_evaluator->lower_bound()));
  EXPECT_TRUE(CompareMatrices(ub, bbox_evaluator->upper_bound()));

  const int N_CONSTRAINTS = 3;
  VectorXd b(N_CONSTRAINTS);
  b << -lb[0], -lb[1], ub[1];  // all the finite lower/upper bounds.

  MatrixXd A(N_CONSTRAINTS, 2);
  // Rows of A:
  // 1. Lower bound y[0]
  // 2. Lower bound y[1]
  // 3. Upper bound y[1]
  A << -1, 0, 0, -1, 0, 1;

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());

  // First linear constraint (in the new decision variables) is 0 ≤
  // (Ay-b)(Ay-b)ᵀ, where A and b represent all of the constraints stacked.
  auto linear_constraint = relaxation->linear_constraints()[0];
  VectorXd value = relaxation->EvalBindingAtInitialGuess(linear_constraint);
  MatrixXd expected_mat =
      (A * y_test - b) * (A * y_test - b).transpose() - b * b.transpose();
  VectorXd expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value = linear_constraint.evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, LinearConstraint) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  MatrixXd A0(3, 2);
  A0 << 0.5, 0.7, -0.2, 0.4, -2.3, -4.5;
  const Vector3d lb0(1.3, -kInf, 0.25);
  const Vector3d ub0(5.6, 0.1, kInf);
  prog.AddLinearConstraint(A0, lb0, ub0, y);
  Matrix2d A1;
  A1 << 0.2, 1.2, 0.24, -0.1;
  const Vector2d lb1(-0.74, -0.3);
  const Vector2d ub1(-0.75, 0.9);
  prog.AddLinearConstraint(A1, lb1, ub1, Vector2<Variable>(y[1], y[0]));
  Matrix2d A1_reordered;
  A1_reordered.col(0) = A1.col(1);
  A1_reordered.col(1) = A1.col(0);

  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_constraints().size(), 3);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 5);

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());

  // First linear constraint is lb0 ≤ A0y ≤ ub0.
  EXPECT_TRUE(CompareMatrices(
      A0, relaxation->linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      lb0, relaxation->linear_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      ub0, relaxation->linear_constraints()[0].evaluator()->upper_bound()));

  // Second linear constraint is lb1 ≤ A1 y ≤ ub1.
  EXPECT_TRUE(CompareMatrices(
      A1, relaxation->linear_constraints()[1].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      lb1, relaxation->linear_constraints()[1].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      ub1, relaxation->linear_constraints()[1].evaluator()->upper_bound()));

  // Third linear (in the new decision variables) constraint is 0 ≤
  // (Ay-b)(Ay-b)ᵀ, where A and b represent all of the constraints stacked.
  VectorXd b(8);  // all of the finite lower/upper bounds.
  b << -lb0[0], ub0[0], ub0[1], -lb0[2], -lb1[0], ub1[0], -lb1[1], ub1[1];
  MatrixXd A(8, 2);
  A << -A0.row(0), A0.row(0), A0.row(1), -A0.row(2), -A1_reordered.row(0),
      A1_reordered.row(0), -A1_reordered.row(1), A1_reordered.row(1);
  int expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation->linear_constraints()[2].evaluator()->num_constraints(),
            expected_size);
  VectorXd value = relaxation->EvalBindingAtInitialGuess(
      relaxation->linear_constraints()[2]);
  MatrixXd expected_mat =
      (A * y_test - b) * (A * y_test - b).transpose() - b * b.transpose();
  VectorXd expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value = relaxation->linear_constraints()[2].evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
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
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 4);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 5);

  const Vector2d y_test(1.3, 0.24);
  SetRelaxationInitialGuess(y_test, relaxation.get());

  // First constraint is (Ay-b)=0.
  MatrixXd expected = A * y_test - b;
  VectorXd value =
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_equality_constraints()[0]) -
      relaxation->linear_equality_constraints()[0].evaluator()->lower_bound();
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));

  // The second constraint is X(-1,-1) = 1.
  expected = Eigen::VectorXd::Ones(1);
  value = relaxation->EvalBindingAtInitialGuess(
      relaxation->linear_equality_constraints()[1]);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));

  for (int i = 2; i < 4; ++i) {
    // Linear constraints are (Ay - b)*y_i = 0.
    expected = (A * y_test - b) * y_test[i - 2];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[i]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
}

GTEST_TEST(MakeSemidefiniteRelaxationTest, NonConvexQuadraticConstraint) {
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
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 3);

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

// This test checks that repeated variables in a quadratic constraint are
// handled correctly.
GTEST_TEST(MakeSemidefiniteRelaxationTest, QuadraticConstraint2) {
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<1>("y");
  prog.AddQuadraticConstraint(Eigen::Matrix2d::Ones(), Eigen::Vector2d::Zero(),
                              0, 1, Vector2<Variable>(y(0), y(0)));
  auto relaxation = MakeSemidefiniteRelaxation(prog);

  EXPECT_EQ(relaxation->num_vars(), 3);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 3);

  const Vector1d y_test(1.3);
  SetRelaxationInitialGuess(y_test, relaxation.get());
  EXPECT_NEAR(relaxation->EvalBindingAtInitialGuess(
                  relaxation->linear_constraints()[0])[0],
              2 * y_test(0) * y_test(0), 1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->lower_bound()[0],
            0.0);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->upper_bound()[0],
            1.0);
}

class MakeSemidefiniteRelaxationVariableGroupTest : public ::testing::Test {
 protected:
  void SetUp() override {
    x_ = prog_.NewContinuousVariables<3>();
    y_ = prog_.NewContinuousVariables<2>();
    partition_group_.emplace_back(x_);
    partition_group_.emplace_back(y_);
    overlap_group_.emplace_back(
        std::initializer_list<symbolic::Variable>({x_(1), y_(0)}));
    overlap_group_.emplace_back(
        std::initializer_list<symbolic::Variable>({x_(0), y_(1)}));
  }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
  VectorIndeterminate<2> y_;

  // A grouping of the variables that partitions the variables of the program.
  std::vector<Variables> partition_group_;
  // A grouping of the variables which overlaps.
  std::vector<Variables> overlap_group_;
};

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, EmptyVariableGroup) {
  // Make prog_ a convex program.
  prog_.AddLinearCost(y_[0]);
  prog_.AddQuadraticCost(y_[0] * y_[0] + y_[1] * y_[1],
                         true /* a convex cost*/);

  prog_.AddLinearConstraint(x_[0] >= x_[1]);
  prog_.AddLinearEqualityConstraint(x_[1] == x_[2]);
  prog_.AddQuadraticConstraint(
      x_[0] * x_[0] + x_[1] * x_[1], -kInf, 1,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);

  // Since prog_ is convex and we have no variable groups, then the relaxation
  // should be exactly the original program.
  const auto relaxation_empty =
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>());

  // The relaxation has the exact same variables as the original program.
  Variables relax_vars{relaxation_empty->decision_variables()};
  Variables prog_vars{prog_.decision_variables()};
  EXPECT_TRUE(relax_vars.IsSubsetOf(prog_vars) &&
              prog_vars.IsSubsetOf(relax_vars));

  // The relaxation has the same costs.
  EXPECT_EQ(relaxation_empty->linear_costs().size(),
            prog_.linear_costs().size());
  EXPECT_EQ(relaxation_empty->quadratic_costs().size(),
            prog_.quadratic_costs().size());
  EXPECT_EQ(relaxation_empty->GetAllCosts().size(), prog_.GetAllCosts().size());

  // The relaxation has the same constraints.
  EXPECT_EQ(relaxation_empty->linear_constraints().size(),
            prog_.linear_constraints().size());
  EXPECT_EQ(relaxation_empty->linear_equality_constraints().size(),
            prog_.linear_equality_constraints().size());
  EXPECT_EQ(relaxation_empty->quadratic_constraints().size(),
            prog_.quadratic_constraints().size());
  EXPECT_EQ(relaxation_empty->GetAllConstraints().size(),
            prog_.GetAllConstraints().size());

  // Now add a non-convex quadratic cost and expect a throw.
  auto non_convex_quadratic_cost = prog_.AddQuadraticCost(x_[0] * y_[1], false);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>()),
      ".*non-convex.*");
  prog_.RemoveCost(non_convex_quadratic_cost);

  // Now add a non-convex quadratic constraint and expect a throw.
  auto non_convex_quadratic_constraint = prog_.AddQuadraticConstraint(
      x_[0] * y_[1], 0, 1, QuadraticConstraint::HessianType::kIndefinite);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>()),
      ".*non-convex.*");
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake

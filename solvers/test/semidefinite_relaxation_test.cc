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

void SetRelaxationInitialGuess(std::map<Variable, double> expected_values,
                               MathematicalProgram* relaxation) {
  for (const auto& [var, val] : expected_values) {
    relaxation->SetInitialGuess(var, val);
  }
  for (const auto& constraint :
       relaxation->positive_semidefinite_constraints()) {
    const int n = constraint.evaluator()->matrix_rows();
    VectorXd x_expected(n);
    const MatrixX<Variable> X_var = Eigen::Map<const MatrixX<Variable>>(
        constraint.variables().data(), n, n);
    for (int i = 0; i < n - 1; ++i) {
      x_expected(i) = expected_values.at(X_var(i, n - 1));
    }
    x_expected(n - 1) = 1;
    const MatrixXd X_expected = x_expected * x_expected.transpose();
    relaxation->SetInitialGuess(X_var, X_expected);
  }
}

int Nchoose2(int n) {
  return (n * (n - 1)) / 2;
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
    x_ = prog_.NewContinuousVariables<3>("x");
    y_ = prog_.NewContinuousVariables<2>("y");
    partition_group_.emplace_back(x_);
    partition_group_.emplace_back(y_);
    overlap_group_.emplace_back(
        std::initializer_list<symbolic::Variable>({x_(0), y_(0)}));
    overlap_group_.emplace_back(
        std::initializer_list<symbolic::Variable>({x_(1), y_(0), y_(1)}));
  }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
  VectorIndeterminate<2> y_;

  // A grouping of the variables that partitions the variables of the program.
  std::vector<Variables> partition_group_;
  // A grouping of the variables which overlaps.
  std::vector<Variables> overlap_group_;
};

// This test checks that constraints and costs which do not intersect any
// variable group do not get relaxed into semidefinite variables.
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

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, LinearCost) {
  const std::map<Variable, double> test_point{
      {x_(0), 1.1}, {x_(1), 0.24}, {x_(2), -2.2}, {y_(0), -0.7}, {y_(1), -3.1}};

  prog_.AddLinearCost(x_[0] + x_[1]);

  auto relaxation_partition =
      MakeSemidefiniteRelaxation(prog_, partition_group_);
  // Only relaxes the x_ variables.
  EXPECT_EQ(relaxation_partition->linear_costs().size(), 1);
  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), x(2), 1]. The remaining
  // variables that do not get relaxed are [y(0), y(1)]. So there are (5
  // choose 2) + 2 variables in the program.
  EXPECT_EQ(relaxation_partition->num_vars(), 2 + Nchoose2(5));
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            1);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());
  EXPECT_NEAR(relaxation_partition->EvalBindingAtInitialGuess(
                  relaxation_partition->linear_costs()[0])[0],
              test_point.at(x_(0)) + test_point.at(x_(1)), 1e-12);

  auto relaxation_overlap = MakeSemidefiniteRelaxation(prog_, overlap_group_);

  // Both variable groups intersect the linear cost, but the cost should only
  // get added once so we still only have 1 PSD variable.
  EXPECT_EQ(relaxation_overlap->linear_costs().size(), 1);
  EXPECT_EQ(relaxation_overlap->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), y(0), 1]. The remaining
  // variables that do not get relaxed are [x(2), y(1)]. So there are (5 choose
  // 2) + 2 variables in the program.
  EXPECT_EQ(relaxation_overlap->num_vars(), Nchoose2(5) + 2);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            overlap_group_[0].size() + 2);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation_overlap->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation_overlap->GetAllConstraints().size(), 2);

  SetRelaxationInitialGuess(test_point, relaxation_overlap.get());
  EXPECT_NEAR(relaxation_overlap->EvalBindingAtInitialGuess(
                  relaxation_overlap->linear_costs()[0])[0],
              test_point.at(x_(0)) + test_point.at(x_(1)), 1e-12);

  // Add a linear cost that intersects the second of the variable groups.
  prog_.AddLinearCost(y_[1]);
  relaxation_overlap = MakeSemidefiniteRelaxation(prog_, overlap_group_);
  // Both variable groups intersect the first linear cost and one of the groups
  // intersects the second cost, we should get two PSD variables, one relaxing
  // [x(0), x(1), y(0), 1] and another relaxing [x(0), x(1), y(0), y(1), 1].
  // There is the remaining x(2) variable that is not relaxed.
  EXPECT_EQ(relaxation_overlap->linear_costs().size(), 2);
  EXPECT_EQ(relaxation_overlap->num_vars(),
            Nchoose2(5) + Nchoose2(6) + 1 -
                3 /* x(0), x(1) and y(0) are double counted so subtract 3*/);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            5);
  // The two equality constraints that the "1" in the psd variables is equal to
  // 1, plus the equality constraint of the psd matrices.
  EXPECT_EQ(relaxation_overlap->linear_equality_constraints().size(), 3);
  EXPECT_EQ(relaxation_overlap->GetAllConstraints().size(), 3 + 2);

  SetRelaxationInitialGuess(test_point, relaxation_overlap.get());
  EXPECT_NEAR(relaxation_overlap->EvalBindingAtInitialGuess(
                  relaxation_overlap->linear_costs()[0])[0],
              test_point.at(x_(0)) + test_point.at(x_(1)), 1e-12);
  EXPECT_NEAR(relaxation_overlap->EvalBindingAtInitialGuess(
                  relaxation_overlap->linear_costs()[1])[0],
              test_point.at(y_(1)), 1e-12);

  // Check that the equality constraint on the relaxation matrices is correct.
  auto psd_agree_constraint =
      relaxation_overlap->linear_equality_constraints()[2];
  EXPECT_TRUE(CompareMatrices(
      relaxation_overlap->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, QuadraticCost) {
  const std::map<Variable, double> test_points{
      {x_(0), 0.1}, {x_(1), -3.24}, {x_(2), 4.2}, {y_(0), -1.7}, {y_(1), -7.7}};

  prog_.AddQuadraticCost(x_[0] * x_[1]);

  auto relaxation_partition =
      MakeSemidefiniteRelaxation(prog_, partition_group_);
  // Only relaxes the x_ variables.
  EXPECT_EQ(relaxation_partition->linear_costs().size(), 1);
  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), x(2), 1]. The remaining
  // variables that do not get relaxed are [y(0), y(1)]. So there are (4
  // choose 2) + 2 variables in the program.
  EXPECT_EQ(relaxation_partition->num_vars(), 2 + Nchoose2(5));
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            1);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2);
  SetRelaxationInitialGuess(test_points, relaxation_partition.get());
  EXPECT_NEAR(relaxation_partition->EvalBindingAtInitialGuess(
                  relaxation_partition->linear_costs()[0])[0],
              test_points.at(x_(0)) * test_points.at(x_(1)), 1e-12);

  auto relaxation_overlap = MakeSemidefiniteRelaxation(prog_, overlap_group_);

  // Both variable groups intersect the linear cost, but the cost should only
  // get added once so we still only have 1 PSD variable.
  EXPECT_EQ(relaxation_overlap->linear_costs().size(), 1);
  EXPECT_EQ(relaxation_overlap->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), y(0), 1]. The remaining
  // variables that do not get relaxed are [x(2), y(1)]. So there are (5 choose
  // 2) + 2 variables in the program.
  EXPECT_EQ(relaxation_overlap->num_vars(), Nchoose2(5) + 2);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            overlap_group_[0].size() + 2);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation_overlap->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation_overlap->GetAllConstraints().size(), 2);
  SetRelaxationInitialGuess(test_points, relaxation_overlap.get());
  EXPECT_NEAR(relaxation_overlap->EvalBindingAtInitialGuess(
                  relaxation_overlap->linear_costs()[0])[0],
              test_points.at(x_(0)) * test_points.at(x_(1)), 1e-12);

  //  Add a quadratic cost that intersects the second of the variable groups.
  prog_.AddQuadraticCost(x_[2] * y_[1]);
  relaxation_overlap = MakeSemidefiniteRelaxation(prog_, overlap_group_);
  // Both variable groups intersect the first linear cost and one of the groups
  // intersects the second cost, we should get two PSD variables, one relaxing
  // [x(0), x(1), y(0), 1] and another relaxing
  // [x(0), x(1), x(2), y(0), y(1), 1].
  EXPECT_EQ(relaxation_overlap->linear_costs().size(), 2);
  EXPECT_EQ(relaxation_overlap->num_vars(),
            Nchoose2(5) + Nchoose2(7) -
                3 /* x(0), x(1) and y(0) are double counted so subtract 2*/);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_overlap->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            6);
  // The two equality constraints that the "1" in the psd variables are equal to
  // 1, plus the PSD variables overlapping in
  EXPECT_EQ(relaxation_overlap->linear_equality_constraints().size(), 3);
  EXPECT_EQ(relaxation_overlap->GetAllConstraints().size(), 3 + 2);

  SetRelaxationInitialGuess(test_points, relaxation_overlap.get());
  EXPECT_NEAR(relaxation_overlap->EvalBindingAtInitialGuess(
                  relaxation_overlap->linear_costs()[0])[0],
              test_points.at(x_(0)) * test_points.at(x_(1)), 1e-12);
  EXPECT_NEAR(relaxation_overlap->EvalBindingAtInitialGuess(
                  relaxation_overlap->linear_costs()[1])[0],
              test_points.at(x_(2)) * test_points.at(y_(1)), 1e-12);

  // Check that the equality constraint on the relaxation matrices is correct.
  auto psd_agree_constraint =
      relaxation_overlap->linear_equality_constraints()[2];
  EXPECT_TRUE(CompareMatrices(
      relaxation_overlap->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, QuadraticConstraint) {
  const std::map<Variable, double> test_point{
      {x_(0), 0.3}, {x_(1), -1.9}, {x_(2), -1.4}, {y_(0), -2.3}, {y_(1), -6.3}};

  // An indefinite Q for the x_ variables.
  // clang-format off
  const Matrix3d Qx{{-3.3,  -0.8,  -0.5},
                    {-1.1,  -1.7, -10.3},
                    { 2.4,   0.3,   1.9}};
  // clang-format on
  const Vector3d bx(-0.2, -3.1, 2.7);
  const double lbx = -.7, ubx = 1.5;
  prog_.AddQuadraticConstraint(Qx, bx, lbx, ubx, x_);

  // A PSD Q for the y_ variables. Q is PSD since it is diagonally dominant.
  // clang-format off
  const Matrix2d Qy{{1.7,  -0.8},
                    {-0.8,   4.6}};
  // clang-format on
  const Vector2d by(-1.3, -0.4);
  const double lby = 1.1, uby = 4.7;
  prog_.AddQuadraticConstraint(
      Qy, by, lby, uby, y_,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);

  // Relaxes the x_ and y_ variables separately.
  auto relaxation_partition =
      MakeSemidefiniteRelaxation(prog_, partition_group_);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());

  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and
  // [y(0), y(1), 1]. So there are (5 choose 2) + (4 choose 2) variables in the
  // program.
  EXPECT_EQ(relaxation_partition->num_vars(), Nchoose2(5) + Nchoose2(4));
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // The two constraints that the "1" in the psd variables are equal to 1.
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(), 2);
  // 2 original quadratic constraints.
  EXPECT_EQ(relaxation_partition->linear_constraints().size(), 2);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2 + 2 + 2);
  const Vector3d x_test(test_point.at(x_(0)), test_point.at(x_(1)),
                        test_point.at(x_(2)));
  EXPECT_NEAR(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition->linear_constraints()[0])[0],
      (0.5 * x_test.transpose() * Qx * x_test + bx.transpose() * x_test)[0],
      1e-12);
  EXPECT_EQ(relaxation_partition->linear_constraints()[0]
                .evaluator()
                ->lower_bound()[0],
            lbx);
  EXPECT_EQ(relaxation_partition->linear_constraints()[0]
                .evaluator()
                ->upper_bound()[0],
            ubx);

  const Vector2d y_test(test_point.at(y_(0)), test_point.at(y_(1)));
  EXPECT_NEAR(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition->linear_constraints()[1])[0],
      (0.5 * y_test.transpose() * Qy * y_test + by.transpose() * y_test)[0],
      1e-12);
  EXPECT_EQ(relaxation_partition->linear_constraints()[1]
                .evaluator()
                ->lower_bound()[0],
            lby);
  EXPECT_EQ(relaxation_partition->linear_constraints()[1]
                .evaluator()
                ->upper_bound()[0],
            uby);

  // Now add a quadratic constraint which intersects both variable groups. This
  // will add 1 linear constraint to each semidefinite variable group, a linear
  // equality constraint on the two semidefinite variables enforcing agreement.
  // clang-format off
  const Matrix2d Q_overlap{{2.3, -4.1},
                           {-7.6,  1.9}};
  // clang-format on
  const Vector2d b_overlap{-1.2, 7.3};
  const double lb_overlap = 0.1, ub_overlap = 2.2;
  VectorXDecisionVariable overlap_vars(2);
  // These are intentionally placed out of order with respect to the sorting
  // that happens DoAddSemidefiniteVariableAndImpliedCostsAndConstraints in
  overlap_vars << y_(1), x_(0);
  prog_.AddQuadraticConstraint(Q_overlap, b_overlap, lb_overlap, ub_overlap,
                               overlap_vars);
  relaxation_partition = MakeSemidefiniteRelaxation(prog_, partition_group_);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());

  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), y(1), 1] and
  // [x(0), y(0), y(1), 1]. These two groups of variables overlap in 2 places.
  // So there are (6 choose 2) + (5 choose 2) - 2 variables in the program
  EXPECT_EQ(relaxation_partition->num_vars(), Nchoose2(6) + Nchoose2(5) - 2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  // The two constraints that the "1" in the psd variables are equal to 1 plus
  // the one constraints that the minors indexed by [x(0), y(1)] are equal.
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(), 3);
  // Check that the linear equality constraints are the claimed ones.
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition->linear_equality_constraints()[0]),
      Eigen::VectorXd::Ones(1)));
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition->linear_equality_constraints()[1]),
      Eigen::VectorXd::Ones(1)));
  auto psd_agree_constraint =
      relaxation_partition->linear_equality_constraints()[2];
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));

  // The first two quadratic constraints each become linear constraints. The
  // third quadratic constraints gets converted to a linear constraint in each
  // relaxation of the subgroups.
  EXPECT_EQ(relaxation_partition->linear_constraints().size(), 4);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2 + 3 + 4);
  VectorXd overlap_test_vec1(4);
  overlap_test_vec1 << test_point.at(x_(0)), test_point.at(x_(1)),
      test_point.at(x_(2)), test_point.at(y_(1));

  // The first quadratic constraint should be the Qx quadratic constraint.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  Q.topLeftCorner(Qx.rows(), Qx.rows()) = Qx;
  Eigen::VectorXd b(4);
  b << bx, 0;
  EXPECT_NEAR(relaxation_partition->EvalBindingAtInitialGuess(
                  relaxation_partition->linear_constraints()[0])[0],
              (0.5 * overlap_test_vec1.transpose() * Q * overlap_test_vec1 +
               b.transpose() * overlap_test_vec1)[0],
              1e-12);
  EXPECT_EQ(relaxation_partition->linear_constraints()[0]
                .evaluator()
                ->lower_bound()[0],
            lbx);
  EXPECT_EQ(relaxation_partition->linear_constraints()[0]
                .evaluator()
                ->upper_bound()[0],
            ubx);
  // The second quadratic constraint should be Q_overlap quadratic constraint.
  Q.setZero();
  Q(0, 0) = Q_overlap(1, 1);
  Q(0, 3) = Q_overlap(1, 0);
  Q(3, 0) = Q_overlap(0, 1);
  Q(3, 3) = Q_overlap(0, 0);
  b.setZero();
  b(0) = b_overlap(1);
  b(3) = b_overlap(0);
  EXPECT_NEAR(relaxation_partition->EvalBindingAtInitialGuess(
                  relaxation_partition->linear_constraints()[1])[0],
              (0.5 * overlap_test_vec1.transpose() * Q * overlap_test_vec1 +
               b.transpose() * overlap_test_vec1)[0],
              1e-12);
  EXPECT_EQ(relaxation_partition->linear_constraints()[1]
                .evaluator()
                ->lower_bound()[0],
            lb_overlap);
  EXPECT_EQ(relaxation_partition->linear_constraints()[1]
                .evaluator()
                ->upper_bound()[0],
            ub_overlap);

  VectorXd overlap_test_vec2(3);
  overlap_test_vec2 << test_point.at(x_(0)), test_point.at(y_(0)),
      test_point.at(y_(1));
  // The third quadratic constraint should be the Qy quadratic constraint.
  Q = Eigen::MatrixXd::Zero(3, 3);
  Q.bottomRightCorner(Qy.rows(), Qy.rows()) = Qy;
  b = Eigen::VectorXd::Zero(3);
  b << 0, by;
  EXPECT_NEAR(relaxation_partition->EvalBindingAtInitialGuess(
                  relaxation_partition->linear_constraints()[2])[0],
              (0.5 * overlap_test_vec2.transpose() * Q * overlap_test_vec2 +
               b.transpose() * overlap_test_vec2)[0],
              1e-12);
  EXPECT_EQ(relaxation_partition->linear_constraints()[2]
                .evaluator()
                ->lower_bound()[0],
            lby);
  EXPECT_EQ(relaxation_partition->linear_constraints()[2]
                .evaluator()
                ->upper_bound()[0],
            uby);
  // The fourth quadratic constraint should be Q_overlap quadratic constraint.
  Q.setZero();
  Q(0, 0) = Q_overlap(1, 1);
  Q(0, 2) = Q_overlap(1, 0);
  Q(2, 0) = Q_overlap(0, 1);
  Q(2, 2) = Q_overlap(0, 0);
  b.setZero();
  b(0) = b_overlap(1);
  b(2) = b_overlap(0);
  EXPECT_NEAR(relaxation_partition->EvalBindingAtInitialGuess(
                  relaxation_partition->linear_constraints()[3])[0],
              (0.5 * overlap_test_vec2.transpose() * Q * overlap_test_vec2 +
               b.transpose() * overlap_test_vec2)[0],
              1e-12);
  EXPECT_EQ(relaxation_partition->linear_constraints()[3]
                .evaluator()
                ->lower_bound()[0],
            lb_overlap);
  EXPECT_EQ(relaxation_partition->linear_constraints()[3]
                .evaluator()
                ->upper_bound()[0],
            ub_overlap);
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, LinearConstraint) {
  const std::map<Variable, double> test_point{
      {x_(0), 1.1}, {x_(1), 0.27}, {x_(2), -1.2}, {y_(0), -0.99}, {y_(1), 9.1}};

  MatrixXd Ax(2, 3);
  // clang-format off
  Ax << 2,     0, 3.1,
        -1, -1.7, 2.1;
  // clang-format on
  const Vector2d lbx(1.3, -kInf);
  const Vector2d ubx(kInf, 0.1);
  prog_.AddLinearConstraint(Ax, lbx, ubx, x_);

  MatrixXd Ay(3, 2);
  // clang-format off
  Ay << 1.1,   2.7,
        0.27, -9.1,
        -1.2, -0.99;
  // clang-format on
  const Vector3d lby(1.3, -kInf, kInf);
  const Vector3d uby(kInf, 0.1, 7.2);
  prog_.AddLinearConstraint(Ay, lby, uby, y_);

  // Relaxes the x_ and y_ variables separately.
  auto relaxation_partition =
      MakeSemidefiniteRelaxation(prog_, partition_group_);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());
  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and
  // [y(0), y(1), 1]. So there are (5 choose 2) + (4 choose 2) variables in the
  // program.
  EXPECT_EQ(relaxation_partition->num_vars(), Nchoose2(5) + Nchoose2(4));
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // The two constraints that the "1" in the psd variables are equal to 1.
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(), 2);
  // 2 original linear constraints, and each of these adds one product
  // constraint.
  EXPECT_EQ(relaxation_partition->linear_constraints().size(), 2 + 2);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2 + 2 + 4);

  // First 2 linear constraints are lbx ≤ Ax * x ≤ ubx, and lby ≤ Ay * y ≤ uby.
  EXPECT_TRUE(CompareMatrices(
      Ax,
      relaxation_partition->linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(lbx, relaxation_partition->linear_constraints()[0]
                                       .evaluator()
                                       ->lower_bound()));
  EXPECT_TRUE(CompareMatrices(ubx, relaxation_partition->linear_constraints()[0]
                                       .evaluator()
                                       ->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      Ay,
      relaxation_partition->linear_constraints()[1].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(lby, relaxation_partition->linear_constraints()[1]
                                       .evaluator()
                                       ->lower_bound()));
  EXPECT_TRUE(CompareMatrices(uby, relaxation_partition->linear_constraints()[1]
                                       .evaluator()
                                       ->upper_bound()));
  // Third linear (in the new decision variables) constraint is 0 ≤
  // (A*x-b)(A*x-b)ᵀ, where A and b represent all of the on x_ constraints
  // stacked.
  VectorXd b(2);  // all of the finite lower/upper bounds.
  b << -lbx[0], ubx[1];
  MatrixXd A(2, 3);
  A << -Ax.row(0), Ax.row(1);
  int expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation_partition->linear_constraints()[2]
                .evaluator()
                ->num_constraints(),
            expected_size);
  const Vector3d x_test(test_point.at(x_(0)), test_point.at(x_(1)),
                        test_point.at(x_(2)));
  VectorXd value = relaxation_partition->EvalBindingAtInitialGuess(
      relaxation_partition->linear_constraints()[2]);
  MatrixXd expected_mat =
      (A * x_test - b) * (A * x_test - b).transpose() - b * b.transpose();
  VectorXd expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value =
      relaxation_partition->linear_constraints()[2].evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));

  // Fourth linear (in the new decision variables) constraint is 0 ≤
  // (A*y-b)(A*y-b)ᵀ, where A and b represent all of the on y_ constraints
  // stacked.
  b.resize(3);  // all of the finite lower/upper bounds.
  b << -lby[0], uby[1], uby[2];
  A.resize(3, 2);
  A << -Ay.row(0), Ay.row(1), Ay.row(2);
  expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation_partition->linear_constraints()[3]
                .evaluator()
                ->num_constraints(),
            expected_size);
  const Vector2d y_test(test_point.at(y_(0)), test_point.at(y_(1)));
  value = relaxation_partition->EvalBindingAtInitialGuess(
      relaxation_partition->linear_constraints()[3]);
  expected_mat =
      (A * y_test - b) * (A * y_test - b).transpose() - b * b.transpose();
  expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value =
      relaxation_partition->linear_constraints()[3].evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));

  // Now add a linear constraint which intersects both variable groups. This
  // will add 2 linear constraints to each semidefinite variable group, and 1
  // linear equality constraint on the two semidefinite variables enforcing
  // agreement on the overlap.
  MatrixXd A_overlap(1, 2);
  // clang-format off
  A_overlap << -1.9, 2.7;
  // clang-format on
  const Vector1d lb_overlap(1.7);
  const Vector1d ub_overlap(2.3);
  VectorXDecisionVariable overlap_vars(2);
  overlap_vars << x_(1), y_(1);
  prog_.AddLinearConstraint(A_overlap, lb_overlap, ub_overlap, overlap_vars);
  relaxation_partition = MakeSemidefiniteRelaxation(prog_, partition_group_);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());

  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), y(1), 1] and
  // [x(1), y(0), y(1), 1]. These two groups of variables overlap in 2 places.
  // So there are (6 choose 2) + (5 choose 2) - 2 variables in the program
  EXPECT_EQ(relaxation_partition->num_vars(), Nchoose2(6) + Nchoose2(5) - 2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  // The two constraints that the "1" in the psd variables are equal to 1 plus
  // the one constraints that the minors indexed by [x(1), y(1)] are equal.
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(), 3);
  // Check that the linear equality constraints are the claimed ones.
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition->linear_equality_constraints()[0]),
      Eigen::VectorXd::Ones(1)));
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition->linear_equality_constraints()[1]),
      Eigen::VectorXd::Ones(1)));
  auto psd_agree_constraint =
      relaxation_partition->linear_equality_constraints()[2];
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));

  // 3 original linear constraints, and each semidefinite relaxation has the
  // product constraints.
  EXPECT_EQ(relaxation_partition->linear_constraints().size(), 3 + 2);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2 + 3 + 5);

  // First 3 linear constraints are lbx ≤ Ax * x ≤ ubx, and lby ≤ Ay * y ≤ uby
  // and lb_overlap ≤ A_overlap * overlap_vars ≤ ub_overlap.
  EXPECT_TRUE(CompareMatrices(
      Ax,
      relaxation_partition->linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(lbx, relaxation_partition->linear_constraints()[0]
                                       .evaluator()
                                       ->lower_bound()));
  EXPECT_TRUE(CompareMatrices(ubx, relaxation_partition->linear_constraints()[0]
                                       .evaluator()
                                       ->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      Ay,
      relaxation_partition->linear_constraints()[1].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(lby, relaxation_partition->linear_constraints()[1]
                                       .evaluator()
                                       ->lower_bound()));
  EXPECT_TRUE(CompareMatrices(uby, relaxation_partition->linear_constraints()[1]
                                       .evaluator()
                                       ->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      A_overlap,
      relaxation_partition->linear_constraints()[2].evaluator()->GetDenseA()));
  EXPECT_TRUE(
      CompareMatrices(lb_overlap, relaxation_partition->linear_constraints()[2]
                                      .evaluator()
                                      ->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(ub_overlap, relaxation_partition->linear_constraints()[2]
                                      .evaluator()
                                      ->upper_bound()));

  // Fourth linear (in the new decision variables) constraint is 0 ≤
  // (A*x-b)(A*x-b)ᵀ, where A and b represent all of the on x_ and the overlap
  // constraints stacked.
  b.resize(4);  // all of the finite lower/upper bounds.
  b << -lbx[0], ubx[1], -lb_overlap[0], ub_overlap[0];
  A.resize(4, 4);
  // clang-format off
  A << -Ax.row(0), 0,
        Ax.row(1), 0,
        0, -A_overlap(0, 0), 0, -A_overlap(0, 1),
        0,  A_overlap(0, 0), 0,  A_overlap(0, 1);
  // clang-format on
  expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation_partition->linear_constraints()[3]
                .evaluator()
                ->num_constraints(),
            expected_size);
  VectorXd test_vec(4);
  test_vec << test_point.at(x_(0)), test_point.at(x_(1)), test_point.at(x_(2)),
      test_point.at(y_(1));
  value = relaxation_partition->EvalBindingAtInitialGuess(
      relaxation_partition->linear_constraints()[3]);
  expected_mat =
      (A * test_vec - b) * (A * test_vec - b).transpose() - b * b.transpose();
  expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value =
      relaxation_partition->linear_constraints()[3].evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));

  // Fifth linear (in the new decision variables) constraint is 0 ≤
  // (A*x-b)(A*x-b)ᵀ, where A and b represent all of the on y_ and the overlap
  // constraints stacked.
  b.resize(5);  // all of the finite lower/upper bounds.
  b << -lby[0], uby[1], uby[2], -lb_overlap[0], ub_overlap[0];
  A.resize(5, 3);
  // clang-format off
  A << 0,              -Ay.row(0),
       0,               Ay.row(1),
       0,               Ay.row(2),
       -A_overlap(0, 0), 0, -A_overlap(0, 1),
        A_overlap(0, 0), 0,  A_overlap(0, 1);
  // clang-format on
  expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation_partition->linear_constraints()[4]
                .evaluator()
                ->num_constraints(),
            expected_size);
  test_vec.resize(3);
  test_vec << test_point.at(x_(1)), test_point.at(y_(0)), test_point.at(y_(1));

  value = relaxation_partition->EvalBindingAtInitialGuess(
      relaxation_partition->linear_constraints()[4]);
  expected_mat =
      (A * test_vec - b) * (A * test_vec - b).transpose() - b * b.transpose();
  expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value =
      relaxation_partition->linear_constraints()[4].evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, LinearEqualityConstraint) {
  const std::map<Variable, double> test_point{
      {x_(0), -0.6}, {x_(1), 3.8}, {x_(2), 4.7}, {y_(0), 9.9}, {y_(1), 3.4}};

  // clang-format off
  const Matrix3d Ax{{0.8, -2.3, 0.7},
                    {-2.1, 2.9, 2.2},
                    {-2.8, 0.5, 5.2}};
  // clang-format on
  const Vector3d bx(3.58, -4.26, -2.61);
  prog_.AddLinearEqualityConstraint(Ax, bx, x_);

  MatrixXd Ay(3, 2);
  // clang-format off
  Ay << -0.08, 1.62,
         5.17, -6.47,
         0.93, -2.97;
  // clang-format on
  const Vector3d by(4.6, -0.1, 0.7);
  const Vector3d uby(kInf, 0.1, 7.2);
  prog_.AddLinearEqualityConstraint(Ay, by, y_);

  // Relaxes the x_ and y_ variables separately.
  auto relaxation_partition =
      MakeSemidefiniteRelaxation(prog_, partition_group_);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());
  EXPECT_EQ(relaxation_partition->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and
  // [y(0), y(1), 1]. So there are (5 choose 2) + (4 choose 2) variables in the
  // program.
  EXPECT_EQ(relaxation_partition->num_vars(), Nchoose2(5) + Nchoose2(4));
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // The two constraints that the "1" in the psd variables are equal to 1.
  // Additionally, the relaxation of each groups causes one extra product
  // constraint for each of the column of the resulting PSD matrices.
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(),
            2 + 4 + 3);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2 + 9);

  int cur_constraint = 0;
  // The first two constraints are the linear constraints inherited from prog_.
  auto CompareLinearEqualityConstraintsAtIdx = [&](const int idx) {
    EXPECT_TRUE(CompareMatrices(
        relaxation_partition->linear_equality_constraints()[idx]
            .evaluator()
            ->GetDenseA(),
        prog_.linear_equality_constraints()[idx].evaluator()->GetDenseA()));
    EXPECT_TRUE(CompareMatrices(
        relaxation_partition->linear_equality_constraints()[idx]
            .evaluator()
            ->lower_bound(),
        prog_.linear_equality_constraints()[idx].evaluator()->lower_bound()));
  };
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  ASSERT_EQ(cur_constraint, 2);

  // This next set of equality constraints are due to the relaxation of the x_
  // variables.
  const Vector3d x_test(test_point.at(x_(0)), test_point.at(x_(1)),
                        test_point.at(x_(2)));
  VectorXd expected;
  VectorXd value;
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition
              ->linear_equality_constraints()[cur_constraint++]),
      Eigen::VectorXd::Ones(1)));
  int start = cur_constraint;
  for (; cur_constraint < start + 3; ++cur_constraint) {
    // Linear constraints are (Ax*x_ - bx)*x_i = 0.
    expected = (Ax * x_test - bx) * x_test[cur_constraint - start];
    value = relaxation_partition->EvalBindingAtInitialGuess(
        relaxation_partition->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  ASSERT_EQ(cur_constraint, 6);

  // This set of equality constraints are due to the relaxation of the y_
  // variables.
  const Vector2d y_test(test_point.at(y_(0)), test_point.at(y_(1)));
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition
              ->linear_equality_constraints()[cur_constraint++]),
      Eigen::VectorXd::Ones(1)));
  start = cur_constraint;
  for (; cur_constraint < start + 2; ++cur_constraint) {
    // Linear constraints are (Ax*x_ - b)*x_i = 0.
    expected = (Ay * y_test - by) * y_test[cur_constraint - start];
    value = relaxation_partition->EvalBindingAtInitialGuess(
        relaxation_partition->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  // At this point, we should have tested all the linear equality constraints.
  ASSERT_EQ(cur_constraint,
            relaxation_partition->linear_equality_constraints().size());

  // Now add a linear equality constraint which intersects both variable groups.
  // This will add 2 linear constraints to each semidefinite variable group, and
  // 1 linear equality constraint on the two semidefinite variables enforcing
  // agreement on the overlap.
  MatrixXd A_overlap(1, 2);
  // clang-format off
  A_overlap << -7.9, 2.4;
  // clang-format on
  const Vector1d b_overlap(0.7);
  VectorXDecisionVariable overlap_vars(2);
  overlap_vars << x_(2), y_(0);
  prog_.AddLinearEqualityConstraint(A_overlap, b_overlap, overlap_vars);

  relaxation_partition = MakeSemidefiniteRelaxation(prog_, partition_group_);
  SetRelaxationInitialGuess(test_point, relaxation_partition.get());

  // The variables which get relaxed are [x(0), x(1), x(2), y(0), 1] and
  // [x(2), y(0), y(1), 1]. These two groups of variables overlap in 2 places.
  // So there are (6 choose 2) + (5 choose 2) - 2 variables in the program
  EXPECT_EQ(relaxation_partition->num_vars(), Nchoose2(6) + Nchoose2(5) - 2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints().size(),
            2);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation_partition->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  // Three linear equality constraints in the original program, then two
  // constraints that the "1" in the psd variables are equal to 1. Additionally,
  // the relaxation of each groups causes one extra product constraint for each
  // linear equality constraint and for each of the column of the resulting PSD
  // matrices minus 1. Finally, one more equality constraint for the agreement
  // of the PSD matrices.
  EXPECT_EQ(relaxation_partition->linear_equality_constraints().size(),
            3 + 2 + 2 * 4 + 2 * 3 + 1);
  EXPECT_EQ(relaxation_partition->GetAllConstraints().size(), 2 + 20);
  cur_constraint = 0;

  // The first three constraints are the linear constraints inherited from
  // prog_.
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  ASSERT_EQ(cur_constraint, 3);

  // This next set of equality constraints are due to the relaxation of [x(0),
  // x(1), x(2), y(0), 1].
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition
              ->linear_equality_constraints()[cur_constraint++]),
      Eigen::VectorXd::Ones(1)));
  const Vector2d overlap_test(test_point.at(x_(2)), test_point.at(y_(0)));

  VectorXd partition1_vec_test(4);
  partition1_vec_test << test_point.at(x_(0)), test_point.at(x_(1)),
      test_point.at(x_(2)), test_point.at(y_(0));

  start = cur_constraint;
  for (; cur_constraint < start + 4; ++cur_constraint) {
    // Linear constraints are (Ax*x_ - bx)*x_i = 0.
    expected = (Ax * x_test - bx) * partition1_vec_test[cur_constraint - start];
    value = relaxation_partition->EvalBindingAtInitialGuess(
        relaxation_partition->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  start = cur_constraint;
  for (; cur_constraint < start + 4; ++cur_constraint) {
    // Linear constraints are (A_overlap*[x_(2), y_(0)] - b_overlap)*x_i = 0.
    expected = (A_overlap * overlap_test - b_overlap) *
               partition1_vec_test[cur_constraint - start];
    value = relaxation_partition->EvalBindingAtInitialGuess(
        relaxation_partition->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  ASSERT_EQ(cur_constraint, 12);

  // This next set of equality constraints are due to the relaxation of [x(2),
  // y(0), y(1), 1].
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(
          relaxation_partition
              ->linear_equality_constraints()[cur_constraint++]),
      Eigen::VectorXd::Ones(1)));

  VectorXd partition2_vec_test(3);
  partition2_vec_test << test_point.at(x_(2)), test_point.at(y_(0)),
      test_point.at(y_(1));

  start = cur_constraint;
  for (; cur_constraint < start + 3; ++cur_constraint) {
    // Linear constraints are (Ay*y_ - by)*partition2_vec_test_i = 0.
    expected = (Ay * y_test - by) * partition2_vec_test[cur_constraint - start];
    value = relaxation_partition->EvalBindingAtInitialGuess(
        relaxation_partition->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  start = cur_constraint;
  for (; cur_constraint < start + 3; ++cur_constraint) {
    // Linear constraints are (A_overlap*[x_(2), y_(0)] -
    // b_overlap)*partition2_vec_test_i = 0.
    expected = (A_overlap * overlap_test - b_overlap) *
               partition2_vec_test[cur_constraint - start];
    value = relaxation_partition->EvalBindingAtInitialGuess(
        relaxation_partition->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }

  ASSERT_EQ(cur_constraint, 19);
  auto psd_agree_constraint =
      relaxation_partition->linear_equality_constraints()[cur_constraint++];
  EXPECT_TRUE(CompareMatrices(
      relaxation_partition->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));
  // All linear equality constraints have been checked.
  ASSERT_EQ(cur_constraint,
            relaxation_partition->linear_equality_constraints().size());
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake

#include "drake/solvers/semidefinite_relaxation.h"

#include <limits>
#include <map>
#include <vector>

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

void SetRelaxationInitialGuess(
    const std::map<Variable, double>& expected_values,
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

int NChoose2(int n) {
  return (n * (n - 1)) / 2;
}

const double kInf = std::numeric_limits<double>::infinity();

}  // namespace

GTEST_TEST(SemidefiniteRelaxationOptions, DefaultOptionsTest) {
  // A change to the default options can have a large effect on both
  // the solve time and tightness of semidefinite relaxations. If the default
  // options are changed, please ensure that the commit message specifically
  // highlights this change for downstream developers.
  SemidefiniteRelaxationOptions options{};
  EXPECT_TRUE(options.add_implied_linear_equality_constraints);
  EXPECT_TRUE(options.add_implied_linear_constraints);
}

GTEST_TEST(SemidefiniteRelaxationOptions, SetWeakestTest) {
  SemidefiniteRelaxationOptions options{};
  options.set_to_weakest();
  EXPECT_FALSE(options.add_implied_linear_equality_constraints);
  EXPECT_FALSE(options.add_implied_linear_constraints);
}

GTEST_TEST(SemidefiniteRelaxationOptions, SetStrongestTest) {
  SemidefiniteRelaxationOptions options{};
  options.set_to_strongest();
  EXPECT_TRUE(options.add_implied_linear_equality_constraints);
  EXPECT_TRUE(options.add_implied_linear_constraints);
}

class MakeSemidefiniteRelaxationTest : public ::testing::Test {
 protected:
  // Construct a program with at least one of every supported attribute for
  // MakeSemidefiniteRelaxation. See ValidateProgramIsSupported for details on
  // the supported attributes.
  void SetUp() override {
    y_ = prog_.NewContinuousVariables<5>("x");

    // LinearCost
    const Vector2d a_cost(0.5, 0.7);
    const double b_cost = 1.3;
    prog_.AddLinearCost(a_cost, b_cost, y_.head<2>());

    // Non-convex QuadraticCost.
    Matrix2d Q_non_convex;
    // clang-format off
    Q_non_convex << 1, 2,
                    3, 4;
    // clang-format on
    const Vector2d b_non_convex(0.2, 0.4);
    prog_.AddQuadraticCost(Q_non_convex, b_non_convex, y_.segment(1, 2), false);

    // Convex QuadraticCost.
    const Vector2d yd(0.5, 0.7);
    prog_.AddQuadraticErrorCost(Matrix2d::Identity(), yd, y_.segment(1, 2));

    // BoundingBoxConstraint
    VectorXd lb(3);
    lb << -1.5, -2.0, -kInf;
    VectorXd ub(3);
    ub << kInf, 2.3, 0;
    prog_.AddBoundingBoxConstraint(lb, ub, y_.head<3>());

    // LinearConstraint.
    MatrixXd A0(3, 2);
    // clang-format off
    A0 <<  1.5,  0.7,
          -0.2,  2.4,
          -2.3, -0.5;
    // clang-format on
    const Vector3d lb0(1.3, -kInf, 0.25);
    const Vector3d ub0(5.6, 0.1, kInf);
    prog_.AddLinearConstraint(A0, lb0, ub0, y_.segment(2, 2));

    // LinearEqualityConstraint.
    MatrixXd A_eq(2, 3);
    // clang-format off
    A_eq <<  0.5,  0.7, -0.2,
             0.4, -2.3, -4.5;
    // clang-format on
    const Vector2d b_eq(1.3, -0.24);
    prog_.AddLinearEqualityConstraint(A_eq, b_eq, y_.tail<3>());

    // Non-convex QuadraticConstraint.
    const double lb_non_convex = -0.4, ub_non_convex = 0.5;
    prog_.AddQuadraticConstraint(Q_non_convex, b_non_convex, lb_non_convex,
                                 ub_non_convex, y_.segment(2, 2));

    // Convex QuadraticConstraint.
    Matrix2d Q_convex;
    // Q_convex is PSD since it is diagonally dominant.
    // clang-format off
    Q_convex << 1,   0.5,
                0.5, 1;
    // clang-format on
    const Vector2d b_convex(0.2, 0.4);
    const double ub_convex = 0.5;
    prog_.AddQuadraticConstraint(Q_convex, b_convex, -kInf, ub_convex,
                                 y_.segment(1, 2));
    ASSERT_TRUE(prog_.quadratic_constraints()[1].evaluator()->is_convex());
  }

  // A program with all the supported costs and constraints
  MathematicalProgram prog_;
  VectorIndeterminate<5> y_;
};

TEST_F(MakeSemidefiniteRelaxationTest, EnsureProgramsAreValidated) {
  SemidefiniteRelaxationOptions options{};
  prog_.AddCost(sin(y_[0]));
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog_, options),
      ".*GenericCost was declared but is not supported.");
}

// This test is used to verify that the initial linear costs and constraints are
// cloned into the relaxation.
TEST_F(MakeSemidefiniteRelaxationTest, VerifyLinearCostsAndConstraintsCloned) {
  // Remove all quadratic costs and constraints.
  const std::vector<Binding<QuadraticCost>> quadratic_costs =
      prog_.quadratic_costs();
  for (const auto& cost : quadratic_costs) {
    prog_.RemoveCost(cost);
  }
  const std::vector<Binding<QuadraticConstraint>> quadratic_constraints =
      prog_.quadratic_constraints();
  for (const auto& constraint : quadratic_constraints) {
    prog_.RemoveConstraint(constraint);
  }

  SemidefiniteRelaxationOptions options{};
  // Don't add any implied constraints.
  options.set_to_weakest();
  auto relaxation = MakeSemidefiniteRelaxation(prog_, options);
  // The semidefinite program is initialized.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  // The linear costs and constraints are cloned.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  EXPECT_EQ(relaxation->linear_constraints().size(), 1);
  EXPECT_EQ(relaxation->bounding_box_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 3 + 1 + 1);

  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);

  VectorXd y_test(5);
  y_test << 1.3, 0.24, 0.5, 0.2, -0.4;
  SetRelaxationInitialGuess(y_test, relaxation.get());

  // The linear cost is correct.
  const VectorXd a_cost = prog_.linear_costs()[0].evaluator()->a();
  const double b_cost = prog_.linear_costs()[0].evaluator()->b();
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      a_cost.transpose() * y_test.head<2>() + b_cost, 1e-12);

  // The linear constraint is correct.
  const MatrixXd A_constraint =
      prog_.linear_constraints()[0].evaluator()->GetDenseA();
  EXPECT_TRUE(CompareMatrices(relaxation->EvalBindingAtInitialGuess(
                                  relaxation->linear_constraints()[0]),
                              A_constraint * y_test.segment(2, 2), 1e-12));
  EXPECT_TRUE(CompareMatrices(
      relaxation->linear_constraints()[0].evaluator()->upper_bound(),
      prog_.linear_constraints()[0].evaluator()->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      relaxation->linear_constraints()[0].evaluator()->lower_bound(),
      prog_.linear_constraints()[0].evaluator()->lower_bound()));

  // The bounding box constraint is correct.
  const MatrixXd A_bb =
      prog_.bounding_box_constraints()[0].evaluator()->GetDenseA();
  EXPECT_TRUE(CompareMatrices(relaxation->EvalBindingAtInitialGuess(
                                  relaxation->bounding_box_constraints()[0]),
                              A_bb.transpose() * y_test.head<3>(), 1e-12));
  EXPECT_TRUE(CompareMatrices(
      relaxation->bounding_box_constraints()[0].evaluator()->upper_bound(),
      prog_.bounding_box_constraints()[0].evaluator()->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      relaxation->bounding_box_constraints()[0].evaluator()->lower_bound(),
      prog_.bounding_box_constraints()[0].evaluator()->lower_bound()));
  // The linear equality constraint is correct.
  const MatrixXd A_eq =
      prog_.linear_equality_constraints()[0].evaluator()->GetDenseA();
  EXPECT_TRUE(CompareMatrices(relaxation->EvalBindingAtInitialGuess(
                                  relaxation->linear_equality_constraints()[0]),
                              A_eq * y_test.tail<3>(), 1e-12));
  EXPECT_TRUE(CompareMatrices(
      relaxation->linear_equality_constraints()[0].evaluator()->upper_bound(),
      prog_.linear_equality_constraints()[0].evaluator()->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      relaxation->linear_equality_constraints()[0].evaluator()->lower_bound(),
      prog_.linear_equality_constraints()[0].evaluator()->lower_bound()));

  // The variable "one" is constrained to be equal to 1.
  const auto one_binding = relaxation->linear_equality_constraints()[1];
  EXPECT_EQ(one_binding.variables().size(), 1);
  EXPECT_EQ(one_binding.variables()[0].get_name(), "one");
  EXPECT_EQ(relaxation->EvalBindingAtInitialGuess(one_binding).size(), 1);
  EXPECT_NEAR(one_binding.evaluator()->upper_bound()[0], 1.0, 1e-12);
  EXPECT_NEAR(one_binding.evaluator()->lower_bound()[0], 1.0, 1e-12);

  // Confirm that the decision variables of prog are also decision variables of
  // the relaxation.
  std::vector<int> indices = relaxation->FindDecisionVariableIndices(y_);
  EXPECT_EQ(indices.size(), y_.size());
}

TEST_F(MakeSemidefiniteRelaxationTest, LinearizeQuadraticCostsAndConstraints) {
  SemidefiniteRelaxationOptions options{};
  options.set_to_weakest();
  auto relaxation = MakeSemidefiniteRelaxation(prog_, options);

  // The semidefinite program is initialized.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);

  // All the quadratic costs are linearized.
  EXPECT_EQ(relaxation->quadratic_costs().size(), 0);
  EXPECT_EQ(relaxation->linear_costs().size(),
            prog_.linear_costs().size() + prog_.quadratic_costs().size());
  // We should have the same total number of costs.
  EXPECT_EQ(relaxation->GetAllCosts().size(),
            prog_.linear_costs().size() + prog_.quadratic_costs().size());

  // All the quadratic constraints are linearized.
  EXPECT_EQ(relaxation->quadratic_constraints().size(), 0);
  EXPECT_EQ(
      relaxation->linear_constraints().size(),
      prog_.linear_constraints().size() + prog_.quadratic_constraints().size());

  // One extra constraint from "one" equals 1, one from the semidefinite
  // constraint.
  EXPECT_EQ(relaxation->GetAllConstraints().size(),
            prog_.GetAllConstraints().size() + 1 + 1);
}

TEST_F(MakeSemidefiniteRelaxationTest, AddImpliedLinearConstraint) {
  SemidefiniteRelaxationOptions options{};
  options.set_to_weakest();
  options.add_implied_linear_constraints = true;
  auto relaxation = MakeSemidefiniteRelaxation(prog_, options);

  // We have one additional linear constraint due to the inclusion of implied
  // linear constraints.
  EXPECT_EQ(relaxation->linear_constraints().size(),
            prog_.linear_constraints().size() +
                prog_.quadratic_constraints().size() + 1);

  const auto implied_linear_constraint =
      relaxation->linear_constraints().back();
  // One linear constraint with 4 finite bounds rows, one bounding box
  // constraint with 4 finite bounds.
  const int aggregated_A_row_size = 4 + 4;
  EXPECT_EQ(implied_linear_constraint.evaluator()->GetDenseA().rows(),
            NChoose2(aggregated_A_row_size + 1));
}

TEST_F(MakeSemidefiniteRelaxationTest, AddImpliedLinearEqualityConstraint) {
  SemidefiniteRelaxationOptions options{};
  options.set_to_weakest();
  options.add_implied_linear_equality_constraints = true;
  auto relaxation = MakeSemidefiniteRelaxation(prog_, options);

  // We have one additional linear equality constraint per variable due to the
  // inclusion of implied linear equality constraints as well as one additional
  // linear equality constraint that the variable "one" equals 1.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(),
            prog_.linear_equality_constraints().size() + 1 +
                prog_.linear_equality_constraints().size() * y_.size());
}

class MakeSemidefiniteRelaxationVariableGroupTest : public ::testing::Test {
 protected:
  void SetUp() override {
    x_ = prog_.NewContinuousVariables<3>("x");
    y_ = prog_.NewContinuousVariables<2>("y");

    x_vars_ = Variables(x_);
    y_vars_ = Variables(y_);
    x0_y0_y1_vars_ = Variables{x_(0), y_(0), y_(1)};
    x0_x2_y1_vars_ = Variables{x_(0), x_(2), y_(1)};
  }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
  VectorIndeterminate<2> y_;

  Variables x_vars_;
  Variables y_vars_;
  Variables x0_y0_y1_vars_;
  Variables x0_x2_y1_vars_;
};

// This test checks that constraints and costs which are not a subset of any
// variable group do not get relaxed. Also checks that non-convex quadratics
// that are not relaxed throw errors.
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
  // should be exactly the original program, with one more variable representing
  // the "1" homogenization variable.
  const auto relaxation_empty =
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>());

  // The relaxation has includes same variables as the original program, as well
  // as the "1" homogenization variable.
  Variables relax_vars{relaxation_empty->decision_variables()};
  Variables prog_vars{prog_.decision_variables()};
  EXPECT_TRUE(prog_vars.IsSubsetOf(relax_vars));
  EXPECT_EQ(prog_vars.size() + 1, relax_vars.size());

  // The relaxation has the same costs.
  EXPECT_EQ(relaxation_empty->linear_costs().size(),
            prog_.linear_costs().size());
  EXPECT_EQ(relaxation_empty->quadratic_costs().size(),
            prog_.quadratic_costs().size());
  EXPECT_EQ(relaxation_empty->GetAllCosts().size(), prog_.GetAllCosts().size());

  // The relaxation has the same constraints except for the 1 homogenization
  // constraint.
  EXPECT_EQ(relaxation_empty->linear_constraints().size(),
            prog_.linear_constraints().size());
  EXPECT_EQ(relaxation_empty->linear_equality_constraints().size(),
            prog_.linear_equality_constraints().size() + 1);
  EXPECT_EQ(relaxation_empty->quadratic_constraints().size(),
            prog_.quadratic_constraints().size());
  // This confirms that we don't have any PSD constraints.
  EXPECT_EQ(relaxation_empty->GetAllConstraints().size(),
            prog_.GetAllConstraints().size() + 1);

  // Now add a non-convex quadratic cost and expect a throw.
  auto non_convex_quadratic_cost = prog_.AddQuadraticCost(x_[0] * y_[1], false);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>()),
      ".*non-convex.*");
  prog_.RemoveCost(non_convex_quadratic_cost);

  // Make sure that after removing the non-convex cost, we can construct the
  // relaxation successfully. This ensures that the next test does not throw for
  // the wrong reason (i.e. forgot to delete the non-convex quadratic cost).
  EXPECT_NO_THROW(MakeSemidefiniteRelaxation(prog_, std::vector<Variables>()));

  // Now add a non-convex quadratic constraint and expect a throw.
  auto non_convex_quadratic_constraint = prog_.AddQuadraticConstraint(
      x_[0] * y_[1], 0, 1, QuadraticConstraint::HessianType::kIndefinite);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>()),
      ".*non-convex.*");
}

// Checks that the number of semidefinite matrix variables is always the same as
// the number of variable groups and that the number of rows of each
// semidefinite matrix variable is the size of the corresponding group + 1.
TEST_F(MakeSemidefiniteRelaxationVariableGroupTest,
       EmptyProgramWithVariableGroups) {
  // Only relaxes the x_ variables.
  auto relaxation = MakeSemidefiniteRelaxation(prog_, std::vector{x_vars_});
  // The variables which get relaxed are [x(0), x(1), x(2), 1]. The remaining
  // variables that do not get relaxed are [y(0), y(1)]. So there are (5
  // choose 2) + 2 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), 2 + NChoose2(5));
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
  const Eigen::MatrixX<Variable> X = Eigen::Map<const MatrixX<Variable>>(
      relaxation->positive_semidefinite_constraints()[0].variables().data(),
      x_.size() + 1, x_.size() + 1);
  for (int i = 0; i < x_.size(); ++i) {
    EXPECT_TRUE(X(i, X.cols() - 1).equal_to(x_[i]));
  }

  relaxation =
      MakeSemidefiniteRelaxation(prog_, std::vector{x0_y0_y1_vars_, y_vars_});
  // The variables which get relaxed are [x(0), y(0), y(1), 1] and [y(0), y(1),
  // 1]. The remaining variables that do not get relaxed are [x(1), x(2)]. The
  // variables [y(0), y(1), 1] get double counted So there are (5 choose 2) + (4
  // choose 2) + 2 - 3 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + NChoose2(4) + 2 - 3);
  // One linear equality constraints from the 1 variables being equal to 1 and
  // one linear equality constraint that the minors corresponding [y0, y1] are
  // the same in the two psd variables.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  // Two psd constraints.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // No costs are added.
  EXPECT_EQ(relaxation->linear_costs().size(), 0);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);

  const std::map<Variable, double> test_point{
      {x_(0), 1.1}, {x_(1), 0.24}, {x_(2), -2.2}, {y_(0), -0.7}, {y_(1), -3.1}};
  SetRelaxationInitialGuess(test_point, relaxation.get());
  // Check the equality constraints are correct. The first one is that "1"
  // equals 1.
  EXPECT_NEAR(relaxation->EvalBindingAtInitialGuess(
                  relaxation->linear_equality_constraints()[0])[0],
              1, 1e-12);
  // The second one is that the submatrices corresponding to [y0, y1] are the
  // same.
  auto constraint = relaxation->linear_equality_constraints()[1];
  const Eigen::MatrixX<Variable> X0 = Eigen::Map<const MatrixX<Variable>>(
      relaxation->positive_semidefinite_constraints()[0].variables().data(),
      x0_y0_y1_vars_.size() + 1, x0_y0_y1_vars_.size() + 1);
  const Eigen::MatrixX<Variable> X1 = Eigen::Map<const MatrixX<Variable>>(
      relaxation->positive_semidefinite_constraints()[1].variables().data(),
      y_vars_.size() + 1, y_vars_.size() + 1);
  Eigen::MatrixXd X0_y0_y1(2, 2);
  Eigen::MatrixXd X1_y0_y1(2, 2);
  // clang-format off
  X0_y0_y1 << relaxation->GetInitialGuess(X0(1, 1)),
      relaxation->GetInitialGuess(X0(1, 2)),
      relaxation->GetInitialGuess(X0(1, 2)),
      relaxation->GetInitialGuess(X0(2, 2));

  X1_y0_y1 << relaxation->GetInitialGuess(X1(0, 0)),
      relaxation->GetInitialGuess(X1(0, 1)),
      relaxation->GetInitialGuess(X1(0, 1)),
      relaxation->GetInitialGuess(X1(1, 1));
  // clang-format on
  EXPECT_TRUE(CompareMatrices(X0_y0_y1, X1_y0_y1));
  const Variables overlap_variables{X0(1, 1), X0(1, 2), X0(2, 2),
                                    X1(0, 0), X1(0, 1), X1(1, 1)};
  EXPECT_EQ(Variables{constraint.variables()}, overlap_variables);
  EXPECT_TRUE(CompareMatrices(relaxation->EvalBindingAtInitialGuess(constraint),
                              Eigen::Vector3d::Zero()));
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, LinearCost) {
  const std::map<Variable, double> test_point{
      {x_(0), 1.1}, {x_(1), 0.24}, {x_(2), -2.2}, {y_(0), -0.7}, {y_(1), -3.1}};

  prog_.AddLinearCost(x_[0] + x_[2] + y_[1]);

  // Only relaxes the x_ variables.
  auto relaxation =
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>{x0_x2_y1_vars_});
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), x(2), 1]. The remaining
  // variables that do not get relaxed are [y(0), y(1)]. So there are (5
  // choose 2) + 2 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), 2 + NChoose2(5));
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      test_point.at(x_(0)) + test_point.at(x_(2)) + test_point.at(y_(1)),
      1e-12);

  // Relax an overlapping set of variables which both have the cost as a subset.
  // The cost should only be added once.
  const Variables x_vars_plus_y1{x_(0), x_(1), x_(2), y_(1)};
  relaxation = MakeSemidefiniteRelaxation(
      prog_, std::vector<Variables>{x_vars_plus_y1, x0_x2_y1_vars_});

  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), x(2), y(1), 1] and [x(0),
  // x(2), y(1), 1]. The remaining variable that does not get relaxed is [y(0)].
  // So there are (6 choose 2) + (5 choose 2) + 1 - 4 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(6) + NChoose2(5) + 1 - 4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 4);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);

  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      test_point.at(x_(0)) + test_point.at(x_(2)) + test_point.at(y_(1)),
      1e-12);
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, QuadraticCost) {
  const std::map<Variable, double> test_point{
      {x_(0), 0.1}, {x_(1), -3.24}, {x_(2), 4.2}, {y_(0), -1.7}, {y_(1), -7.7}};

  auto cost = prog_.AddQuadraticCost(x_[0] * x_[1]);

  // Only relaxes the x_ variables.
  auto relaxation =
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>{x_vars_});
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), x(2), 1]. The remaining
  // variables that do not get relaxed are [y(0), y(1)]. So there are (5
  // choose 2) + 2 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), 2 + NChoose2(5));
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      test_point.at(x_(0)) * test_point.at(x_(1)), 1e-12);

  // Relax an overlapping set of variables. The cost should only be added once.
  const Variables y_vars_plus_x0_x1{x_(0), x_(1), y_(0), y_(1)};
  relaxation = MakeSemidefiniteRelaxation(
      prog_, std::vector<Variables>{x_vars_, y_vars_plus_x0_x1});

  // Both variable groups contain the quadratic cost, but the cost should only
  // get added once.
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and [x(0), x(1),
  // y(0), y(1), 1]. The remaining variable that does not get relaxed is [x(2)].
  // So there are (5 choose 2) + (6 choose 2) + 1 - 4 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + NChoose2(6) + 1 - 4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 4);
  EXPECT_EQ(relaxation->linear_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(relaxation->linear_costs()[0])[0],
      test_point.at(x_(0)) * test_point.at(x_(1)), 1e-12);

  // Now we test that a convex quadratic cost isn't relaxed to a linear cost if
  // there is no semidefinite variable which contains the variables of the cost.
  prog_.RemoveCost(cost);
  cost = prog_.AddQuadraticCost(y_(0) * y_(0));
  relaxation =
      MakeSemidefiniteRelaxation(prog_, std::vector<Variables>{x_vars_});

  // The variables which get relaxed are [x(0), x(1), x(2), 1]. The remaining
  // variable that does not get relaxed is [y(0), y(1)]. So there are
  // (5 choose 2) + 2 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2);
  // The only cost is a single quadratic cost.
  EXPECT_EQ(relaxation->quadratic_costs().size(), 1);
  EXPECT_EQ(relaxation->GetAllCosts().size(), 1);
  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_NEAR(relaxation->EvalBindingAtInitialGuess(
                  relaxation->quadratic_costs()[0])[0],
              test_point.at(y_(0)) * test_point.at(y_(0)), 1e-12);
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, QuadraticConstraint) {
  const std::map<Variable, double> test_point{
      {x_(0), 0.3}, {x_(1), -1.9}, {x_(2), -1.4}, {y_(0), -2.3}, {y_(1), -6.3}};

  // An indefinite Q for the (x_(0), x_(2)) variables.
  // clang-format off
  const Matrix2d Qx{{-3.3, -0.8},
                    {-1.1, -1.7}};
  // clang-format on
  const Vector2d bx(-0.2, -3.1);
  const double lbx = -0.7, ubx = 1.5;
  // These are intentionally placed out of order with respect to the
  // ordering that happens in MakeSemidefiniteRelaxation.
  VectorX<Variable> out_of_order_x_vars(2);
  out_of_order_x_vars << x_(2), x_(0);
  prog_.AddQuadraticConstraint(Qx, bx, lbx, ubx, out_of_order_x_vars);

  // A convex quadratic constraint for the y_ variables. Q is PSD since it is
  // diagonally dominant.
  // clang-format off
  const Matrix2d Qy{{1.7, -0.8},
                    {-0.8, 4.6}};
  // clang-format on
  const Vector2d by(-1.3, -0.4);
  const double lby = -kInf, uby = 4.7;
  prog_.AddQuadraticConstraint(
      Qy, by, lby, uby, y_,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);

  // Relaxes the x_ and y_ variables separately.
  auto relaxation = MakeSemidefiniteRelaxation(
      prog_, std::vector<Variables>{x_vars_, y_vars_});
  SetRelaxationInitialGuess(test_point, relaxation.get());

  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and
  // [y(0), y(1), 1]. So there are (5 choose 2) + (4 choose 2) - 1 variables in
  // the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + NChoose2(4) - 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // The constraints that the "1" in the psd variables is equal to 1.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  // 2 original quadratic constraints.
  EXPECT_EQ(relaxation->linear_constraints().size(), 2);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2 + 1 + 2);
  const Vector2d x_test(test_point.at(x_(2)), test_point.at(x_(0)));
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[0])[0],
      (0.5 * x_test.transpose() * Qx * x_test + bx.transpose() * x_test)[0],
      1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->lower_bound()[0],
            lbx);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->upper_bound()[0],
            ubx);

  const Vector2d y_test(test_point.at(y_(0)), test_point.at(y_(1)));
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[1])[0],
      (0.5 * y_test.transpose() * Qy * y_test + by.transpose() * y_test)[0],
      1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[1].evaluator()->lower_bound()[0],
            lby);
  EXPECT_EQ(relaxation->linear_constraints()[1].evaluator()->upper_bound()[0],
            uby);

  // These variable groups are both supersets of both the quadratic constraints.
  // Ensure that the quadratic constraints are added on each variable group, and
  // that the linear equality constraints between the variable groups are added.
  std::vector<Variables> groups{
      Variables{x_(0), x_(2), y_(0), y_(1)},
      Variables{x_(0), x_(1), x_(2), y_(0), y_(1)},
  };
  relaxation = MakeSemidefiniteRelaxation(prog_, groups);
  SetRelaxationInitialGuess(test_point, relaxation.get());

  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x_(0), x_(2), y_(0), y_(1), 1] and
  // [x_(0), x_(1), x_(2), y_(0), y_(1), 1]. The variables [x_(0), x_(2), y_(0),
  // y_(1), 1] get double counted So there are (6 choose 2) + (7 choose 2) - 5
  // variables in the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(6) + NChoose2(7) - 5);
  // One linear equality constraints from the 1 variables being equal to 1 and
  // one linear equality constraint that the minors corresponding [x_(0), x_(2),
  // y_(0), y_(1)] are the same in the two psd variables.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  // Two psd constraints.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            6);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            5);
  // Both quadratic constraints get added to both variable groups as linear
  // constraints.
  EXPECT_EQ(relaxation->linear_constraints().size(), 4);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2 + 2 + 4);

  // Check the linear constraints.
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[0])[0],
      (0.5 * x_test.transpose() * Qx * x_test + bx.transpose() * x_test)[0],
      1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->lower_bound()[0],
            lbx);
  EXPECT_EQ(relaxation->linear_constraints()[0].evaluator()->upper_bound()[0],
            ubx);
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[1])[0],
      (0.5 * y_test.transpose() * Qy * y_test + by.transpose() * y_test)[0],
      1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[1].evaluator()->lower_bound()[0],
            lby);
  EXPECT_EQ(relaxation->linear_constraints()[1].evaluator()->upper_bound()[0],
            uby);

  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[2])[0],
      (0.5 * x_test.transpose() * Qx * x_test + bx.transpose() * x_test)[0],
      1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[2].evaluator()->lower_bound()[0],
            lbx);
  EXPECT_EQ(relaxation->linear_constraints()[2].evaluator()->upper_bound()[0],
            ubx);
  EXPECT_NEAR(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_constraints()[3])[0],
      (0.5 * y_test.transpose() * Qy * y_test + by.transpose() * y_test)[0],
      1e-12);
  EXPECT_EQ(relaxation->linear_constraints()[3].evaluator()->lower_bound()[0],
            lby);
  EXPECT_EQ(relaxation->linear_constraints()[3].evaluator()->upper_bound()[0],
            uby);

  // Now do the relaxation so that both variable group relaxes the first
  // quadratic constraint, but neither variable group relaxes the second
  // quadratic constraint.
  relaxation = MakeSemidefiniteRelaxation(
      prog_, std::vector<Variables>{x_vars_, x0_x2_y1_vars_});

  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x_(0), x_(1), x_(2), 1] and
  // [x_(0), x_(2), y_(1), 1]. The variable y_(0) is not part of any group and
  // [x_(0), x_(2), 1] get double counted So there are (5 choose 2) + (5
  // choose 2) + 1 - 3 variables in the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + NChoose2(5) + 1 - 3);
  // One linear equality constraints from the 1 variables being equal to 1 and
  // one linear equality constraint that the minors corresponding [x_(0), x_(2)]
  // are the same in the two psd variables.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  // Two psd constraints.
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  // The first quadratic constraints get added to both variable groups as a
  // linear constraint.
  EXPECT_EQ(relaxation->linear_constraints().size(), 2);
  // The second quadratic constraint does not get relaxed.
  EXPECT_EQ(relaxation->quadratic_constraints().size(), 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2 + 2 + 2 + 1);
}

TEST_F(MakeSemidefiniteRelaxationVariableGroupTest, LinearConstraint) {
  const std::map<Variable, double> test_point{
      {x_(0), 1.1}, {x_(1), 0.27}, {x_(2), -1.2}, {y_(0), -0.99}, {y_(1), 9.1}};

  MatrixXd Ax(2, 3);
  // clang-format off
  Ax << 2, 0, 3.1,
      -1, -1.7, 2.1;
  // clang-format on
  const Vector2d lbx(1.3, -kInf);
  const Vector2d ubx(kInf, 0.1);
  VectorX<Variable> x_out_of_order(3);
  x_out_of_order << x_(1), x_(0), x_(2);
  prog_.AddLinearConstraint(Ax, lbx, ubx, x_out_of_order);

  MatrixXd Ay(3, 2);
  // clang-format off
  Ay << 1.1, 2.7,
      0.27, -9.1,
      -1.2, -0.99;
  // clang-format on
  const Vector3d lby(1.3, -kInf, kInf);
  const Vector3d uby(kInf, 0.1, 7.2);
  VectorX<Variable> y_out_of_order(2);
  y_out_of_order << y_(1), y_(0);
  prog_.AddLinearConstraint(Ay, lby, uby, y_out_of_order);

  // Relaxes the x_ and y_ variables separately.
  auto relaxation = MakeSemidefiniteRelaxation(
      prog_, std::vector<Variables>{x_vars_, y_vars_});

  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and
  // [y(0), y(1), 1]. So there are (5 choose 2) + (4 choose 2) - 1 variables in
  // the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + NChoose2(4) - 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // The constraint that the "1" in the psd variables is equal to 1.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1);
  // Each variable group is a superset of only 1 of the original linear
  // constraints, each of which gets relaxed into one product constraint.
  // Therefore, there are 4 total linear constraints.
  EXPECT_EQ(relaxation->linear_constraints().size(), 4);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2 + 1 + 4);

  // First 2 linear constraints are lbx ≤ Ax * x ≤ ubx, and lby ≤ Ay * y ≤ uby.
  EXPECT_TRUE(CompareMatrices(
      Ax, relaxation->linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      lbx, relaxation->linear_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      ubx, relaxation->linear_constraints()[0].evaluator()->upper_bound()));
  EXPECT_TRUE(CompareMatrices(
      Ay, relaxation->linear_constraints()[1].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      lby, relaxation->linear_constraints()[1].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      uby, relaxation->linear_constraints()[1].evaluator()->upper_bound()));
  // Third linear (in the new decision variables) constraint is 0 ≤
  // (A*x_-b)(A*x_-b)ᵀ, where A and b represent all of the on x_ constraints
  // stacked.
  VectorXd b(2);  // all of the finite lower/upper bounds.
  b << -lbx[0], ubx[1];
  MatrixXd A(2, 3);
  A << -Ax.row(0), Ax.row(1);
  int expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation->linear_constraints()[2].evaluator()->num_constraints(),
            expected_size);
  const Vector3d x_test(test_point.at(x_(1)), test_point.at(x_(0)),
                        test_point.at(x_(2)));
  VectorXd value = relaxation->EvalBindingAtInitialGuess(
      relaxation->linear_constraints()[2]);
  MatrixXd expected_mat =
      (A * x_test - b) * (A * x_test - b).transpose() - b * b.transpose();
  VectorXd expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value = relaxation->linear_constraints()[2].evaluator()->lower_bound();
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
  EXPECT_EQ(relaxation->linear_constraints()[3].evaluator()->num_constraints(),
            expected_size);
  const Vector2d y_test(test_point.at(y_(1)), test_point.at(y_(0)));
  value = relaxation->EvalBindingAtInitialGuess(
      relaxation->linear_constraints()[3]);
  expected_mat =
      (A * y_test - b) * (A * y_test - b).transpose() - b * b.transpose();
  expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value = relaxation->linear_constraints()[3].evaluator()->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));

  // We add a third linear constraint and use variable groups which overlap on
  // this third constraint only.
  MatrixXd A_overlap(1, 2);
  // clang-format off
  A_overlap << -1.9, 2.7;
  // clang-format on
  const Vector1d lb_overlap(1.7);
  const Vector1d ub_overlap(2.3);
  VectorX<Variable> overlap_vars(2);
  overlap_vars << x_(1), y_(1);
  prog_.AddLinearConstraint(A_overlap, lb_overlap, ub_overlap, overlap_vars);
  std::vector<Variables> groups{Variables{x_(0), x_(1), x_(2), y_(1)},
                                Variables{x_(1), y_(0), y_(1)}};
  relaxation = MakeSemidefiniteRelaxation(prog_, groups);
  SetRelaxationInitialGuess(test_point, relaxation.get());

  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), y(1), 1] and
  // [x(1), y(0), y(1), 1]. These two groups of variables overlap in 3 places.
  // So there are (6 choose 2) + (5 choose 2) - 3 variables in the program
  EXPECT_EQ(relaxation->num_vars(), NChoose2(6) + NChoose2(5) - 3);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  // The constraint that the "1" in the psd variables are equal to 1 plus
  // the one constraints that the minors indexed by [x(1), y(1)] are equal.
  //  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  //  Check that the linear equality constraints are the claimed ones.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 2);
  EXPECT_TRUE(CompareMatrices(relaxation->EvalBindingAtInitialGuess(
                                  relaxation->linear_equality_constraints()[0]),
                              Eigen::VectorXd::Ones(1)));
  auto psd_agree_constraint = relaxation->linear_equality_constraints()[1];
  EXPECT_TRUE(CompareMatrices(
      relaxation->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));

  // 3 original linear constraints and 2 big linear constraints for the product
  // constraints.
  EXPECT_EQ(relaxation->linear_constraints().size(), 3 + 2);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2 + 2 + 5);

  // First 3 linear constraints are lbx ≤ Ax * x ≤ ubx, and lby ≤ Ay * y ≤ uby
  // and lb_overlap ≤ A_overlap * overlap_vars ≤ ub_overlap.
  int constraint_ind = 0;
  EXPECT_TRUE(
      CompareMatrices(Ax, relaxation->linear_constraints()[constraint_ind]
                              .evaluator()
                              ->GetDenseA()));
  EXPECT_TRUE(
      CompareMatrices(lbx, relaxation->linear_constraints()[constraint_ind]
                               .evaluator()
                               ->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(ubx, relaxation->linear_constraints()[constraint_ind]
                               .evaluator()
                               ->upper_bound()));
  ++constraint_ind;
  EXPECT_TRUE(
      CompareMatrices(Ay, relaxation->linear_constraints()[constraint_ind]
                              .evaluator()
                              ->GetDenseA()));
  EXPECT_TRUE(
      CompareMatrices(lby, relaxation->linear_constraints()[constraint_ind]
                               .evaluator()
                               ->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(uby, relaxation->linear_constraints()[constraint_ind]
                               .evaluator()
                               ->upper_bound()));
  ++constraint_ind;
  EXPECT_TRUE(CompareMatrices(A_overlap,
                              relaxation->linear_constraints()[constraint_ind]
                                  .evaluator()
                                  ->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(lb_overlap,
                              relaxation->linear_constraints()[constraint_ind]
                                  .evaluator()
                                  ->lower_bound()));
  EXPECT_TRUE(CompareMatrices(ub_overlap,
                              relaxation->linear_constraints()[constraint_ind]
                                  .evaluator()
                                  ->upper_bound()));
  ++constraint_ind;
  ASSERT_EQ(constraint_ind, 3);

  // Fourth linear (in the new decision variables) constraint is 0 ≤
  // (A*x-b)(A*x-b)ᵀ, where A and b represent all of the on x_ and the overlap
  // constraints stacked.
  b.resize(4);  // all of the finite lower/upper bounds.
  b << -lbx[0], ubx[1], -lb_overlap[0], ub_overlap[0];
  A.resize(4, 4);
  // We need to reshuffle to columns of Ax, as they are out of order.
  // clang-format off
  A << -Ax(0,1), -Ax(0,0),         -Ax(0,2), 0, // NOLINT
        Ax(1,1),  Ax(1,0),          Ax(1,2), 0, // NOLINT
        0,        -A_overlap(0, 0), 0,      -A_overlap(0, 1), // NOLINT
        0,        A_overlap(0, 0),  0,       A_overlap(0, 1); // NOLINT
  // clang-format on
  expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation->linear_constraints()[constraint_ind]
                .evaluator()
                ->num_constraints(),
            expected_size);
  VectorXd test_vec(4);
  test_vec << test_point.at(x_(0)), test_point.at(x_(1)), test_point.at(x_(2)),
      test_point.at(y_(1));
  value = relaxation->EvalBindingAtInitialGuess(
      relaxation->linear_constraints()[constraint_ind]);
  expected_mat =
      (A * test_vec - b) * (A * test_vec - b).transpose() - b * b.transpose();
  expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value = relaxation->linear_constraints()[constraint_ind]
              .evaluator()
              ->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  ++constraint_ind;

  // Fifth linear (in the new decision variables) constraint is 0 ≤
  // (A*x-b)(A*x-b)ᵀ, where A and b represent all of the on y_ and the overlap
  // constraints stacked.
  b.resize(5);  // all of the finite lower/upper bounds.
  b << -lby[0], uby[1], uby[2], -lb_overlap[0], ub_overlap[0];
  A.resize(5, 3);
  // We need to reshuffle to columns of Ay, as they are out of order.
  // clang-format off
  A << 0,               -Ay(0,1), -Ay(0,0), // NOLINT
       0,                Ay(1,1),  Ay(1,0), // NOLINT
       0,                Ay(2,1),  Ay(2,0), // NOLINT
      -A_overlap(0, 0),  0,       -A_overlap(0, 1), // NOLINT
       A_overlap(0, 0),  0,        A_overlap(0, 1); // NOLINT
  // clang-format on

  expected_size = (b.size() * (b.size() + 1)) / 2;
  EXPECT_EQ(relaxation->linear_constraints()[constraint_ind]
                .evaluator()
                ->num_constraints(),
            expected_size);
  test_vec.resize(3);
  test_vec << test_point.at(x_(1)), test_point.at(y_(0)), test_point.at(y_(1));

  value = relaxation->EvalBindingAtInitialGuess(
      relaxation->linear_constraints()[constraint_ind]);
  expected_mat =
      (A * test_vec - b) * (A * test_vec - b).transpose() - b * b.transpose();
  expected = math::ToLowerTriangularColumnsFromMatrix(expected_mat);
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  value = relaxation->linear_constraints()[constraint_ind]
              .evaluator()
              ->lower_bound();
  expected = math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());
  EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  ++constraint_ind;
  ASSERT_EQ(constraint_ind, 5);
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
  auto relaxation = MakeSemidefiniteRelaxation(
      prog_, std::vector<Variables>{x_vars_, y_vars_});
  SetRelaxationInitialGuess(test_point, relaxation.get());
  EXPECT_EQ(relaxation->GetAllCosts().size(), 0);
  // The variables which get relaxed are [x(0), x(1), x(2), 1] and
  // [y(0), y(1), 1]. So there are (5 choose 2) + (4 choose 2) - 1 variables in
  // the program.
  EXPECT_EQ(relaxation->num_vars(), NChoose2(5) + NChoose2(4) - 1);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            4);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            3);
  // The  constraint that the "1" in the psd variables is equal to 1.
  // Additionally, the relaxation of each groups causes one extra product
  // constraint for each of the column of the resulting PSD matrices.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(), 1 + 4 + 3);
  EXPECT_EQ(relaxation->GetAllConstraints().size(), 2 + 8);

  int cur_constraint = 0;
  // The first two constraints are the linear constraints inherited from prog_.
  auto CompareLinearEqualityConstraintsAtIdx = [&](const int idx) {
    EXPECT_TRUE(CompareMatrices(
        relaxation->linear_equality_constraints()[idx].evaluator()->GetDenseA(),
        prog_.linear_equality_constraints()[idx].evaluator()->GetDenseA()));
    EXPECT_TRUE(CompareMatrices(
        relaxation->linear_equality_constraints()[idx]
            .evaluator()
            ->lower_bound(),
        prog_.linear_equality_constraints()[idx].evaluator()->lower_bound()));
  };
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  ASSERT_EQ(cur_constraint, 2);

  // This equality constraint is due to "1" being equal to 1.
  EXPECT_TRUE(CompareMatrices(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_equality_constraints()[cur_constraint++]),
      Eigen::VectorXd::Ones(1)));

  // This next set of equality constraints are due to the relaxation of the x_
  // variables.
  const Vector3d x_test(test_point.at(x_(0)), test_point.at(x_(1)),
                        test_point.at(x_(2)));
  VectorXd expected;
  VectorXd value;
  int start = cur_constraint;
  for (; cur_constraint < start + 3; ++cur_constraint) {
    // Linear constraints are (Ax*x_ - bx)*x_i = 0.
    expected = (Ax * x_test - bx) * x_test[cur_constraint - start];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  ASSERT_EQ(cur_constraint, 6);

  // This set of equality constraints are due to the relaxation of the y_
  // variables.
  const Vector2d y_test(test_point.at(y_(0)), test_point.at(y_(1)));
  start = cur_constraint;
  for (; cur_constraint < start + 2; ++cur_constraint) {
    // Linear constraints are (Ay*y_ - b)*y_i = 0.
    expected = (Ay * y_test - by) * y_test[cur_constraint - start];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  // At this point, we should have tested all the linear equality constraints.
  ASSERT_EQ(cur_constraint, relaxation->linear_equality_constraints().size());

  // Now construct variable groups and equality constraints which overlap.
  MatrixXd A_overlap(1, 2);
  // clang-format off
  A_overlap << -7.9, 2.4;
  // clang-format on
  const Vector1d b_overlap(0.7);
  VectorXDecisionVariable overlap_vars(2);
  // These variables are not sorted according to their id intentionally.
  overlap_vars << y_(0), x_(2);
  const Vector2d overlap_test_vec{test_point.at(y_(0)), test_point.at(x_(2))};
  prog_.AddLinearEqualityConstraint(A_overlap, b_overlap, overlap_vars);

  MatrixXd A_overlap_sorted(A_overlap.rows(), A_overlap.cols());
  A_overlap_sorted << A_overlap(0, 1), A_overlap(0, 0);
  VectorXDecisionVariable overlap_vars_sorted(2);
  overlap_vars_sorted << x_(2), y_(0);
  const Vector2d overlap_test_vec_sorted(test_point.at(x_(2)),
                                         test_point.at(y_(0)));
  // Make sure the sorted and non-sorted version of this constraint are the
  // same.
  EXPECT_TRUE(
      CompareMatrices(A_overlap * overlap_test_vec - b_overlap,
                      A_overlap_sorted * overlap_test_vec_sorted - b_overlap));

  std::vector<Variables> groups{Variables{x_(0), x_(1), x_(2), y_(0)},
                                Variables{x_(2), y_(0), y_(1)}};
  VectorXd group_1_test_vec(4);
  group_1_test_vec << test_point.at(x_(0)), test_point.at(x_(1)),
      test_point.at(x_(2)), test_point.at(y_(0));
  VectorXd group_2_test_vec(3);
  group_2_test_vec << test_point.at(x_(2)), test_point.at(y_(0)),
      test_point.at(y_(1));

  relaxation = MakeSemidefiniteRelaxation(prog_, groups);
  SetRelaxationInitialGuess(test_point, relaxation.get());
  cur_constraint = 0;

  // The variables which get relaxed are [x(0), x(1), x(2), y(0), 1] and
  // [x(2), y(0), y(1), 1]. These two groups of variables overlap in 3 places.
  // So there are (6 choose 2) + (5 choose 2) - 3 variables in the program
  EXPECT_EQ(relaxation->num_vars(), NChoose2(6) + NChoose2(5) - 3);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints().size(), 2);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[0]
                .evaluator()
                ->matrix_rows(),
            5);
  EXPECT_EQ(relaxation->positive_semidefinite_constraints()[1]
                .evaluator()
                ->matrix_rows(),
            4);
  // Three linear equality constraints in the original program, then one
  // constraint that the "1" in the psd variables are equal to 1. Additionally,
  // the relaxation of each groups causes one extra product constraint for each
  // linear equality constraint and for each of the column of the
  // resulting PSD  matrices minus 1. Finally, one more equality constraint for
  // the agreement of the PSD matrices.
  EXPECT_EQ(relaxation->linear_equality_constraints().size(),
            3 + 1 + 2 * 4 + 2 * 3 + 1);
  EXPECT_EQ(relaxation->GetAllConstraints().size(),
            relaxation->positive_semidefinite_constraints().size() +
                relaxation->linear_equality_constraints().size());

  // The first three constraints are the linear constraints inherited from
  // prog_.
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  CompareLinearEqualityConstraintsAtIdx(cur_constraint++);
  ASSERT_EQ(cur_constraint, 3);

  // This next set of equality constraints are due to the relaxation of
  // [x(0), x(1), x(2), y(0), 1].
  EXPECT_TRUE(CompareMatrices(
      relaxation->EvalBindingAtInitialGuess(
          relaxation->linear_equality_constraints()[cur_constraint++]),
      Eigen::VectorXd::Ones(1)));

  start = cur_constraint;
  for (; cur_constraint < start + 4; ++cur_constraint) {
    // Linear constraints are (Ax*x_ - bx)*x_i = 0.
    expected = (Ax * x_test - bx) * group_1_test_vec[cur_constraint - start];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  start = cur_constraint;
  for (; cur_constraint < start + 4; ++cur_constraint) {
    // Linear constraints are (A_overlap*[x_(2), y_(0)] - b_overlap)*[x(0),
    // x(1), x(2), y(0)] = 0.
    expected = (A_overlap_sorted * overlap_test_vec_sorted - b_overlap) *
               group_1_test_vec[cur_constraint - start];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  ASSERT_EQ(cur_constraint, 12);

  // This next set of equality constraints are due to the relaxation of
  // [x(2), y(0), y(1), 1].
  start = cur_constraint;
  for (; cur_constraint < start + 3; ++cur_constraint) {
    // Linear constraints are (Ay*y_ - by)*[x(2), y(0), y(1)]. = 0.
    expected = (Ay * y_test - by) * group_2_test_vec[cur_constraint - start];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  start = cur_constraint;
  for (; cur_constraint < start + 3; ++cur_constraint) {
    // Linear constraints are (A_overlap*[x_(2), y_(0)] -
    // b_overlap)*[x_(2), y_(0), y_(1)] = 0.
    expected = (A_overlap_sorted * overlap_test_vec_sorted - b_overlap) *
               group_2_test_vec[cur_constraint - start];
    value = relaxation->EvalBindingAtInitialGuess(
        relaxation->linear_equality_constraints()[cur_constraint]);
    EXPECT_TRUE(CompareMatrices(value, expected, 1e-12));
  }
  ASSERT_EQ(cur_constraint, 18);

  auto psd_agree_constraint =
      relaxation->linear_equality_constraints()[cur_constraint++];
  EXPECT_TRUE(CompareMatrices(
      relaxation->EvalBindingAtInitialGuess(psd_agree_constraint),
      Eigen::VectorXd::Zero(
          psd_agree_constraint.evaluator()->num_constraints())));
  // All linear equality constraints have been checked.
  ASSERT_EQ(cur_constraint, relaxation->linear_equality_constraints().size());
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake

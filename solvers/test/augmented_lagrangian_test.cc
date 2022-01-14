#include "drake/solvers/augmented_lagrangian.h"

#include <limits>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <gtest/gtest.h>

#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {
const double kInf = std::numeric_limits<double>::infinity();
GTEST_TEST(AugmentedLagrangian, TestObjective) {
  // Construct an un-constrained program.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  Eigen::Vector2d x_val(1, 2);
  // Now the program has no cost nor constraint. Its augmented Lagrangian is
  // zero.
  EXPECT_EQ(EvalAugmentedLagrangian<double>(prog, x_val, Eigen::VectorXd(0),
                                            0.5, false),
            0);

  // Now add costs
  auto cost1 = prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] + 3);
  auto cost2 = prog.AddLinearCost(x[0] * 3 + 4);

  // Now evaluate the augmented Lagrangian, it should just be the same as the
  // costs.
  EXPECT_EQ(
      EvalAugmentedLagrangian<double>(prog, x_val, Eigen::VectorXd(0), 0.5,
                                      false),
      prog.EvalBinding(cost1, x_val)(0) + prog.EvalBinding(cost2, x_val)(0));

  // Now evaluate the augmented Lagrangian with autodiff.
  const auto x_ad = math::InitializeAutoDiff(x_val);
  const auto al = EvalAugmentedLagrangian<AutoDiffXd>(
      prog, x_ad, Eigen::VectorXd(0), 0.5, false);
  const auto al_expected =
      prog.EvalBinding(cost1, x_ad)(0) + prog.EvalBinding(cost2, x_ad)(0);
  EXPECT_EQ(ExtractDoubleOrThrow(al), ExtractDoubleOrThrow(al_expected));
  EXPECT_TRUE(CompareMatrices(al.derivatives(), al_expected.derivatives()));
}

class DummyConstraint final : public solvers::Constraint {
 public:
  DummyConstraint()
      : Constraint(4, 2, Eigen::Vector4d(1, 2, 3, 4),
                   Eigen::Vector4d(1, 2, 3, 4)) {}
  using Constraint::set_bounds;

 protected:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const {
    y->resize(num_constraints());
    (*y)(0) = x(0) * x(1) + 1;
    (*y)(1) = x(0) + x(1);
    (*y)(2) = 2 * x(0) + 3;
    (*y)(3) = 2 * x(0) / x(1) + 4;
  }
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric<double>(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric<AutoDiffXd>(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
  }
};

// This function is only used inside the test
// AugmentedLagrangian.EqualityConstraints. Ideally I want to put it inside the
// test as a templated lambda, but our current compiler on Bionic doesn't
// support this feature yet.
template <typename T>
void CheckAugmentedLagrangianEqualityConstraint(
    const MathematicalProgram& prog, const Binding<Constraint>& constraint,
    const Vector3<T>& x_val, const Eigen::Vector4d& lambda_val, double mu_val,
    double tol_val) {
  const T al =
      EvalAugmentedLagrangian<T>(prog, x_val, lambda_val, mu_val, false);
  const auto cnstr_val = prog.EvalBinding(constraint, x_val);
  const auto& cnstr_bnd = constraint.evaluator()->lower_bound();
  const T al_expected =
      -lambda_val.dot(cnstr_val - cnstr_bnd) +
      1. / (2 * mu_val) * (cnstr_val - cnstr_bnd).dot(cnstr_val - cnstr_bnd);
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_NEAR(al, al_expected, tol_val);
  } else {
    EXPECT_NEAR(ExtractDoubleOrThrow(al), ExtractDoubleOrThrow(al_expected),
                tol_val);
    EXPECT_TRUE(
        CompareMatrices(al.derivatives(), al_expected.derivatives(), tol_val));
  }
}

GTEST_TEST(AugmentedLagrangian, EqualityConstraints) {
  // Test a program with on equality constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto cnstr =
      prog.AddConstraint(std::make_shared<DummyConstraint>(), x.head<2>());

  const double tol = 1E-10;
  CheckAugmentedLagrangianEqualityConstraint(
      prog, cnstr, Eigen::Vector3d(0.5, 1.5, 2), Eigen::Vector4d(1, 3, -2, 4),
      0.2, tol);
  CheckAugmentedLagrangianEqualityConstraint(
      prog, cnstr, Eigen::Vector3d(-0.5, 2.5, 2), Eigen::Vector4d(-1, 2, 3, -2),
      0.6, tol);
  CheckAugmentedLagrangianEqualityConstraint(
      prog, cnstr, math::InitializeAutoDiff(Eigen::Vector3d(0.5, 0.3, 0.2)),
      Eigen::Vector4d(1, 2, -3, -1), 0.5, tol);
}

// Refer to equation 17.55 of Numerical Optimization by Jorge Nocedal and
// Stephen Wright. We use the alternative way to compute s first, and then
// compute psi.
template <typename T>
T psi(const T& c, double lambda, double mu) {
  T s = c - mu * lambda;
  if (ExtractDoubleOrThrow(s) < 0) {
    s = T(0);
  }
  return -lambda * (c - s) + 1 / (2 * mu) * (c - s) * (c - s);
}

template <typename T>
void CheckAugmentedLagrangianInequalityConstraint(
    const MathematicalProgram& prog, const Binding<Constraint>& constraint,
    const Vector3<T>& x_val, const Eigen::Matrix<double, 5, 1>& lambda_val,
    double mu_val, double tol_val) {
  const T al =
      EvalAugmentedLagrangian<T>(prog, x_val, lambda_val, mu_val, false);
  const auto cnstr_val = prog.EvalBinding(constraint, x_val);
  const auto& lb = constraint.evaluator()->lower_bound();
  const auto& ub = constraint.evaluator()->upper_bound();
  using std::pow;
  T al_expected = -lambda_val(0) * (cnstr_val(0) - lb(0)) +
                  1. / (2 * mu_val) * pow(cnstr_val(0) - lb(0), 2) +
                  psi(cnstr_val(1) - lb(1), lambda_val(1), mu_val) +
                  psi(ub(1) - cnstr_val(1), lambda_val(2), mu_val) +
                  psi(ub(2) - cnstr_val(2), lambda_val(3), mu_val) +
                  psi(cnstr_val(3) - lb(3), lambda_val(4), mu_val);
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_NEAR(al, al_expected, tol_val);
  } else {
    EXPECT_NEAR(ExtractDoubleOrThrow(al), ExtractDoubleOrThrow(al_expected),
                tol_val);
    EXPECT_TRUE(
        CompareMatrices(al.derivatives(), al_expected.derivatives(), tol_val));
  }
}
GTEST_TEST(AugmentedLagrangian, InequalityConstraint) {
  // Test with inequality constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto cnstr_evaluator = std::make_shared<DummyConstraint>();
  cnstr_evaluator->set_bounds(Eigen::Vector4d(1, -1, -kInf, 2),
                              Eigen::Vector4d(1, 2, 3, kInf));
  auto cnstr = prog.AddConstraint(cnstr_evaluator, x.tail<2>());

  const double tol = 1E-10;
  CheckAugmentedLagrangianInequalityConstraint(
      prog, cnstr, Eigen::Vector3d(1, 3, 2),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.4, 1.5, 0.2, 3).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianInequalityConstraint(
      prog, cnstr, Eigen::Vector3d(-1, 3, -5),
      (Eigen::Matrix<double, 5, 1>() << -.5, 1.4, 1.5, 0.5, 3).finished(), 0.2,
      tol);
  CheckAugmentedLagrangianInequalityConstraint(
      prog, cnstr, math::InitializeAutoDiff(Eigen::Vector3d(-1, 3, -5)),
      (Eigen::Matrix<double, 5, 1>() << -.5, 1.4, 1.5, 0.5, 3).finished(), 0.2,
      tol);
}

template <typename T>
void CheckAugmentedLagrangianBoundingBoxConstraint(
    const MathematicalProgram& prog, const Vector4<T>& x_val,
    const Eigen::Matrix<double, 5, 1>& lambda, double mu, double tol_val) {
  const T al = EvalAugmentedLagrangian<T>(prog, x_val, lambda, mu, true);
  Eigen::VectorXd x_lb, x_ub;
  AggregateBoundingBoxConstraints(prog, &x_lb, &x_ub);
  const T al_expected =
      -lambda(0) * (x_val(0) - x_lb(0)) +
      1. / (2 * mu) * (x_val(0) - x_lb(0)) * (x_val(0) - x_lb(0)) +
      psi(x_val(1) - x_lb(1), lambda(1), mu) +
      psi(x_ub(2) - x_val(2), lambda(2), mu) +
      psi(x_val(3) - x_lb(3), lambda(3), mu) +
      psi(x_ub(3) - x_val(3), lambda(4), mu);
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_NEAR(al, al_expected, tol_val);
  } else {
    EXPECT_NEAR(al.value(), al_expected.value(), tol_val);
    EXPECT_TRUE(
        CompareMatrices(al.derivatives(), al_expected.derivatives(), tol_val));
  }
}

GTEST_TEST(EvalAugmentedLagrangian, BoundingBoxConstraint) {
  // Test with bounding box constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  auto cnstr1 = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d(1, 3, -kInf, 0), Eigen::Vector4d(2, kInf, -1, 5), x);
  auto cnstr2 = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d(2, 1, -kInf, 1), Eigen::Vector4d(5, kInf, -1, 3), x);

  const double tol = 1E-10;
  CheckAugmentedLagrangianBoundingBoxConstraint(
      prog, Eigen::Vector4d(1, 3, 5, 2),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.3, 1.5, 2, 1).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianBoundingBoxConstraint(
      prog, math::InitializeAutoDiff(Eigen::Vector4d(1, 3, 5, 2)),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.3, 1.5, 2, 1).finished(), 0.5,
      tol);

  // Test include_x_bounds = false, the augmented lagrangian is 0.
  EXPECT_EQ(EvalAugmentedLagrangian<double>(prog, Eigen::Vector4d(1, 2, 3, 4),
                                            Eigen::VectorXd(0), 0.5, false),
            0);
}
}  // namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake

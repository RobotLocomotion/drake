#include "drake/solvers/augmented_lagrangian.h"

#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
const double kInf = std::numeric_limits<double>::infinity();

// This function is only used in AugmentedLagrangian.TestEmptyProg.
template <typename AL>
void TestEmptyProg(const AL& dut) {
  EXPECT_EQ(dut.lagrangian_size(), 0);
  EXPECT_TRUE(dut.is_equality().empty());

  Eigen::Vector2d x_val(1, 2);
  // Now the program has no cost nor constraint. Its augmented Lagrangian is
  // zero.
  Eigen::VectorXd constraint_residue;
  double cost;
  double al_val;
  if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
    al_val = dut.template Eval<double>(x_val, Eigen::VectorXd(0), 0.5,
                                       &constraint_residue, &cost);
  } else {
    al_val =
        dut.template Eval<double>(x_val, Eigen::VectorXd(0), Eigen::VectorXd(0),
                                  0.5, &constraint_residue, &cost);
  }
  EXPECT_EQ(al_val, 0);
  EXPECT_EQ(constraint_residue.rows(), 0);
  EXPECT_EQ(cost, 0);
  EXPECT_TRUE(CompareMatrices(dut.x_lo(), Eigen::Vector2d::Constant(-kInf)));
  EXPECT_TRUE(CompareMatrices(dut.x_up(), Eigen::Vector2d::Constant(kInf)));
}

GTEST_TEST(AugmentedLagrangian, TestEmptyProg) {
  // Construct an un-constrained program.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  const AugmentedLagrangianNonsmooth dut_nonsmooth(&prog, false);
  TestEmptyProg(dut_nonsmooth);
  const AugmentedLagrangianSmooth dut_smooth(&prog, false);
  TestEmptyProg(dut_smooth);
}

template <typename AL>
void TestObjective(const AL& dut) {
  EXPECT_EQ(dut.lagrangian_size(), 0);
  EXPECT_TRUE(dut.is_equality().empty());

  // Now evaluate the augmented Lagrangian, it should just be the same as the
  // costs.
  Eigen::Vector2d x_val(1, 2);
  double cost_expected = 0;
  for (const auto& binding : dut.prog().GetAllCosts()) {
    cost_expected += dut.prog().EvalBinding(binding, x_val)(0);
  }
  Eigen::VectorXd constraint_residue;
  double cost;
  double al_val;
  if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
    al_val = dut.template Eval<double>(x_val, Eigen::VectorXd(0), 0.5,
                                       &constraint_residue, &cost);
  } else {
    al_val =
        dut.template Eval<double>(x_val, Eigen::VectorXd(0), Eigen::VectorXd(0),
                                  0.5, &constraint_residue, &cost);
  }
  EXPECT_EQ(al_val, cost_expected);
  EXPECT_EQ(cost, cost_expected);
  EXPECT_EQ(constraint_residue.rows(), 0);

  // Now evaluate the augmented Lagrangian with autodiff.
  VectorX<AutoDiffXd> constraint_residue_ad;
  AutoDiffXd cost_ad;
  AutoDiffXd al_ad;
  const auto x_ad = math::InitializeAutoDiff(x_val);
  if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
    al_ad = dut.template Eval<AutoDiffXd>(x_ad, Eigen::VectorXd(0), 0.5,
                                          &constraint_residue_ad, &cost_ad);
  } else {
    al_ad = dut.template Eval<AutoDiffXd>(x_ad, AutoDiffVecXd(0),
                                          Eigen::VectorXd(0), 0.5,
                                          &constraint_residue_ad, &cost_ad);
  }
  AutoDiffXd al_expected(0);
  for (const auto& binding : dut.prog().GetAllCosts()) {
    al_expected += dut.prog().EvalBinding(binding, x_ad)(0);
  }
  EXPECT_EQ(ExtractDoubleOrThrow(al_ad), ExtractDoubleOrThrow(al_expected));
  EXPECT_TRUE(CompareMatrices(al_ad.derivatives(), al_expected.derivatives()));
  EXPECT_EQ(cost_ad.value(), al_expected.value());
  EXPECT_TRUE(
      CompareMatrices(cost_ad.derivatives(), al_expected.derivatives()));
  EXPECT_EQ(constraint_residue_ad.rows(), 0);
}

GTEST_TEST(AugmentedLagrangian, TestObjective1) {
  // Test with an empty program.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  for (bool include_x_bounds : {false, true}) {
    const AugmentedLagrangianNonsmooth dut_nonsmooth(&prog, include_x_bounds);
    TestObjective(dut_nonsmooth);
    const AugmentedLagrangianSmooth dut_smooth(&prog, include_x_bounds);
    TestObjective(dut_smooth);
  }
}

GTEST_TEST(AugmentedLagrangian, TestObjective2) {
  // Test a program with costs but no constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Now add costs
  auto cost1 = prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] + 3);
  auto cost2 = prog.AddLinearCost(x[0] * 3 + 4);
  for (bool include_x_bounds : {false, true}) {
    const AugmentedLagrangianNonsmooth dut_nonsmooth(&prog, include_x_bounds);
    TestObjective(dut_nonsmooth);
    const AugmentedLagrangianSmooth dut_smooth(&prog, include_x_bounds);
    TestObjective(dut_smooth);
  }
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

template <typename T>
void CompareAlResult(const T& al, const T& al_expected,
                     const VectorX<T>& constraint_residue,
                     const VectorX<T>& constraint_residue_expected,
                     const T& cost, const T& cost_expected, double tol) {
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_NEAR(al, al_expected, tol);
    EXPECT_TRUE(
        CompareMatrices(constraint_residue, constraint_residue_expected, tol));
    EXPECT_NEAR(cost, cost_expected, tol);
  } else {
    EXPECT_NEAR(al.value(), al_expected.value(), tol);
    EXPECT_TRUE(
        CompareMatrices(al.derivatives(), al_expected.derivatives(), tol));
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(constraint_residue),
                                math::ExtractValue(constraint_residue_expected),
                                tol));
    EXPECT_TRUE(CompareMatrices(
        math::ExtractGradient(constraint_residue),
        math::ExtractGradient(constraint_residue_expected), tol));
    EXPECT_NEAR(cost.value(), cost_expected.value(), tol);
    EXPECT_TRUE(
        CompareMatrices(cost.derivatives(), cost_expected.derivatives(), tol));
  }
}

template <typename T>
T al_equality(const Eigen::Ref<const VectorX<T>>& lhs,
              const Eigen::Ref<const Eigen::VectorXd>& lambda, double mu) {
  return -lambda.dot(lhs) + mu / 2 * lhs.array().square().sum();
}

// This function is only used inside the test
// AugmentedLagrangianNonsmooth.EqualityConstraints. Next time we are making
// any changes here, consider moving it to live directly within that test case
// as a templated lambda.
template <typename T>
void CheckAugmentedLagrangianEqualityConstraint(
    const MathematicalProgram& prog, const Binding<Constraint>& constraint,
    const Vector3<T>& x_val, const Eigen::Vector4d& lambda_val, double mu_val,
    double tol_val, bool smooth_flag) {
  VectorX<T> constraint_residue;
  T cost;
  for (bool include_x_bounds : {false, true}) {
    T al_val;
    if (smooth_flag) {
      const AugmentedLagrangianSmooth dut(&prog, include_x_bounds);
      EXPECT_EQ(dut.lagrangian_size(), 4);
      EXPECT_EQ(dut.is_equality(), std::vector<bool>({true, true, true, true}));
      EXPECT_EQ(dut.s_size(), 0);
      al_val = dut.Eval<T>(x_val, VectorX<T>(0), lambda_val, mu_val,
                           &constraint_residue, &cost);
    } else {
      const AugmentedLagrangianNonsmooth dut(&prog, include_x_bounds);
      EXPECT_EQ(dut.lagrangian_size(), 4);
      EXPECT_EQ(dut.is_equality(), std::vector<bool>({true, true, true, true}));
      al_val =
          dut.Eval<T>(x_val, lambda_val, mu_val, &constraint_residue, &cost);
    }
    const auto constraint_val = prog.EvalBinding(constraint, x_val);
    const auto& constraint_bound = constraint.evaluator()->lower_bound();
    const T al_expected =
        al_equality<T>(constraint_val - constraint_bound, lambda_val, mu_val);
    EXPECT_EQ(constraint_residue.rows(), lambda_val.rows());
    CompareAlResult<T>(al_val, al_expected, constraint_residue,
                       constraint_val - constraint_bound, cost, T{0}, tol_val);
  }
}

GTEST_TEST(AugmentedLagrangian, EqualityConstraints) {
  // Test a program with on equality constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto constraint =
      prog.AddConstraint(std::make_shared<DummyConstraint>(), x.head<2>());

  const double tol = 1E-10;
  for (bool smooth_flag : {false, true}) {
    CheckAugmentedLagrangianEqualityConstraint(
        prog, constraint, Eigen::Vector3d(0.5, 1.5, 2),
        Eigen::Vector4d(1, 3, -2, 4), 0.2, tol, smooth_flag);
    CheckAugmentedLagrangianEqualityConstraint(
        prog, constraint, Eigen::Vector3d(-0.5, 2.5, 2),
        Eigen::Vector4d(-1, 2, 3, -2), 0.6, tol, smooth_flag);
    CheckAugmentedLagrangianEqualityConstraint(
        prog, constraint,
        math::InitializeAutoDiff(Eigen::Vector3d(0.5, 0.3, 0.2)),
        Eigen::Vector4d(1, 2, -3, -1), 0.5, tol, smooth_flag);
  }
}

// Refer to equation 17.55 of Numerical Optimization by Jorge Nocedal and
// Stephen Wright, Edition 1, 1999 (This equation is not presented in Edition
// 2). We use the alternative way to compute s first, and then compute psi. Note
// that what we use for mu here is 1/μ in equation 17.55.
template <typename T>
T psi(const T& c, double lambda, double mu) {
  T s = c - lambda / mu;
  if (ExtractDoubleOrThrow(s) < 0) {
    s = T(0);
  }
  return -lambda * (c - s) + mu / 2 * (c - s) * (c - s);
}

template <typename T>
void CheckAugmentedLagrangianNonsmoothInequalityConstraint(
    const MathematicalProgram& prog, const Binding<Constraint>& constraint,
    const Vector3<T>& x_val, const Eigen::Matrix<double, 5, 1>& lambda_val,
    double mu_val, double tol_val) {
  VectorX<T> constraint_residue;
  T cost;
  for (const bool include_x_bounds : {false, true}) {
    const AugmentedLagrangianNonsmooth dut(&prog, include_x_bounds);
    EXPECT_EQ(dut.lagrangian_size(), 5);
    EXPECT_EQ(dut.is_equality(),
              std::vector<bool>({true, false, false, false, false}));

    const T al =
        dut.Eval<T>(x_val, lambda_val, mu_val, &constraint_residue, &cost);
    const auto constraint_val = prog.EvalBinding(constraint, x_val);
    const auto& lb = constraint.evaluator()->lower_bound();
    const auto& ub = constraint.evaluator()->upper_bound();
    using std::pow;
    T al_expected = -lambda_val(0) * (constraint_val(0) - lb(0)) +
                    mu_val / 2 * pow(constraint_val(0) - lb(0), 2) +
                    psi(constraint_val(1) - lb(1), lambda_val(1), mu_val) +
                    psi(ub(1) - constraint_val(1), lambda_val(2), mu_val) +
                    psi(ub(2) - constraint_val(2), lambda_val(3), mu_val) +
                    psi(constraint_val(3) - lb(3), lambda_val(4), mu_val);
    EXPECT_EQ(constraint_residue.rows(), lambda_val.rows());
    VectorX<T> constraint_residue_expected(constraint_residue.rows());
    constraint_residue_expected << constraint_val(0) - lb(0),
        constraint_val(1) - lb(1), ub(1) - constraint_val(1),
        ub(2) - constraint_val(2), constraint_val(3) - lb(3);
    CompareAlResult<T>(al, al_expected, constraint_residue,
                       constraint_residue_expected, cost, T(0), tol_val);
  }
}

template <typename T>
void CheckAugmentedLagrangianSmoothInequalityConstraint(
    const MathematicalProgram& prog, const Binding<Constraint>& constraint,
    const Vector3<T>& x_val, const Vector4<T>& s_val,
    const Eigen::Matrix<double, 5, 1>& lambda_val, double mu_val,
    double tol_val) {
  VectorX<T> constraint_residue;
  T cost;
  for (const bool include_x_bounds : {false, true}) {
    const AugmentedLagrangianSmooth dut(&prog, include_x_bounds);
    EXPECT_EQ(dut.lagrangian_size(), 5);
    EXPECT_EQ(dut.s_size(), 4);
    EXPECT_EQ(dut.is_equality(),
              std::vector<bool>({true, false, false, false, false}));

    const T al = dut.Eval<T>(x_val, s_val, lambda_val, mu_val,
                             &constraint_residue, &cost);
    const auto constraint_val = prog.EvalBinding(constraint, x_val);
    const auto& lb = constraint.evaluator()->lower_bound();
    const auto& ub = constraint.evaluator()->upper_bound();
    using std::pow;
    Eigen::Matrix<T, 5, 1> lhs;
    lhs << constraint_val(0) - lb(0), constraint_val(1) - lb(1) - s_val(0),
        ub(1) - constraint_val(1) - s_val(1),
        ub(2) - constraint_val(2) - s_val(2),
        constraint_val(3) - lb(3) - s_val(3);
    T al_expected = al_equality<T>(lhs, lambda_val, mu_val);
    EXPECT_EQ(constraint_residue.rows(), lambda_val.rows());
    VectorX<T> constraint_residue_expected = lhs;
    CompareAlResult<T>(al, al_expected, constraint_residue,
                       constraint_residue_expected, cost, T(0), tol_val);
  }
}

GTEST_TEST(AugmentedLagrangian, InequalityConstraint) {
  // Test with inequality constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto constraint_evaluator = std::make_shared<DummyConstraint>();
  constraint_evaluator->set_bounds(Eigen::Vector4d(1, -1, -kInf, 2),
                                   Eigen::Vector4d(1, 2, 3, kInf));
  auto constraint = prog.AddConstraint(constraint_evaluator, x.tail<2>());

  const double tol = 1E-10;
  CheckAugmentedLagrangianNonsmoothInequalityConstraint(
      prog, constraint, Eigen::Vector3d(1, 3, 2),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.4, 1.5, 0.2, 3).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianNonsmoothInequalityConstraint(
      prog, constraint, Eigen::Vector3d(-1, 3, -5),
      (Eigen::Matrix<double, 5, 1>() << -.5, 1.4, 1.5, 0.5, 3).finished(), 0.2,
      tol);
  CheckAugmentedLagrangianNonsmoothInequalityConstraint(
      prog, constraint, math::InitializeAutoDiff(Eigen::Vector3d(-1, 3, -5)),
      (Eigen::Matrix<double, 5, 1>() << -.5, 1.4, 1.5, 0.5, 3).finished(), 0.2,
      tol);

  CheckAugmentedLagrangianSmoothInequalityConstraint(
      prog, constraint, Eigen::Vector3d(1, 3, 2), Eigen::Vector4d(3, -1, -2, 1),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.4, 1.5, 0.2, 3).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianSmoothInequalityConstraint(
      prog, constraint, math::InitializeAutoDiff(Eigen::Vector3d(1, 3, 2)),
      Vector4<AutoDiffXd>(AutoDiffXd(3), AutoDiffXd(-1), AutoDiffXd(-2),
                          AutoDiffXd(1)),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.4, 1.5, 0.2, 3).finished(), 0.5,
      tol);
}

template <typename T>
void CheckAugmentedLagrangianNonsmoothBoundingBoxConstraint(
    const MathematicalProgram& prog, const Vector4<T>& x_val,
    const Eigen::Matrix<double, 5, 1>& lambda, double mu, double tol_val) {
  const AugmentedLagrangianNonsmooth dut(&prog, true);
  EXPECT_EQ(dut.lagrangian_size(), 5);
  EXPECT_EQ(dut.is_equality(),
            std::vector<bool>({true, false, false, false, false}));
  VectorX<T> constraint_residue;
  T cost;
  const T al = dut.Eval<T>(x_val, lambda, mu, &constraint_residue, &cost);
  EXPECT_EQ(constraint_residue.rows(), lambda.rows());
  Eigen::VectorXd x_lb, x_ub;
  AggregateBoundingBoxConstraints(prog, &x_lb, &x_ub);
  const T al_expected = -lambda(0) * (x_val(0) - x_lb(0)) +
                        mu / 2 * (x_val(0) - x_lb(0)) * (x_val(0) - x_lb(0)) +
                        psi(x_val(1) - x_lb(1), lambda(1), mu) +
                        psi(x_ub(2) - x_val(2), lambda(2), mu) +
                        psi(x_val(3) - x_lb(3), lambda(3), mu) +
                        psi(x_ub(3) - x_val(3), lambda(4), mu);
  Eigen::Matrix<T, 5, 1> constraint_residue_expected;
  constraint_residue_expected << x_val(0) - x_lb(0), x_val(1) - x_lb(1),
      x_ub(2) - x_val(2), x_val(3) - x_lb(3), x_ub(3) - x_val(3);
  CompareAlResult<T>(al, al_expected, constraint_residue,
                     constraint_residue_expected, cost, T(0), tol_val);
  Eigen::VectorXd x_lo_expected, x_up_expected;
  AggregateBoundingBoxConstraints(prog, &x_lo_expected, &x_up_expected);
  EXPECT_TRUE(CompareMatrices(dut.x_lo(), x_lo_expected));
  EXPECT_TRUE(CompareMatrices(dut.x_up(), x_up_expected));
}

template <typename T>
void CheckAugmentedLagrangianSmoothBoundingBoxConstraint(
    const MathematicalProgram& prog, const Vector4<T>& x_val,
    const Vector4<T>& s_val, const Eigen::Matrix<double, 5, 1>& lambda,
    double mu, double tol_val) {
  const AugmentedLagrangianSmooth dut(&prog, true);
  EXPECT_EQ(dut.lagrangian_size(), 5);
  EXPECT_EQ(dut.s_size(), 4);
  EXPECT_EQ(dut.is_equality(),
            std::vector<bool>({true, false, false, false, false}));
  VectorX<T> constraint_residue;
  T cost;
  const T al =
      dut.Eval<T>(x_val, s_val, lambda, mu, &constraint_residue, &cost);
  EXPECT_EQ(constraint_residue.rows(), lambda.rows());
  Eigen::VectorXd x_lb, x_ub;
  AggregateBoundingBoxConstraints(prog, &x_lb, &x_ub);
  Eigen::Matrix<T, 5, 1> lhs;
  lhs << x_val(0) - x_lb(0), x_val(1) - x_lb(1) - s_val(0),
      x_ub(2) - x_val(2) - s_val(1), x_val(3) - x_lb(3) - s_val(2),
      x_ub(3) - x_val(3) - s_val(3);
  const T al_expected = al_equality<T>(lhs, lambda, mu);
  Eigen::Matrix<T, 5, 1> constraint_residue_expected = lhs;
  CompareAlResult<T>(al, al_expected, constraint_residue,
                     constraint_residue_expected, cost, T(0), tol_val);
  Eigen::VectorXd x_lo_expected, x_up_expected;
  AggregateBoundingBoxConstraints(prog, &x_lo_expected, &x_up_expected);
  EXPECT_TRUE(CompareMatrices(dut.x_lo(), x_lo_expected));
  EXPECT_TRUE(CompareMatrices(dut.x_up(), x_up_expected));
}

// This function should only be called inside
// EvalAugmentedLagrangian.BoundingBoxConstraint when include_x_bounds=false.
template <typename AL>
void CheckBoundingBoxConstraintEmpty(const AL& dut) {
  DRAKE_DEMAND(dut.include_x_bounds() == false);
  EXPECT_EQ(dut.lagrangian_size(), 0);
  EXPECT_TRUE(dut.is_equality().empty());
  VectorX<double> constraint_residue;
  double cost;
  double al_val;
  if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
    al_val = dut.template Eval<double>(Eigen::Vector4d(1, 2, 3, 4),
                                       Eigen::VectorXd(0), 0.5,
                                       &constraint_residue, &cost);
  } else {
    al_val = dut.template Eval<double>(Eigen::Vector4d(1, 2, 3, 4),
                                       Eigen::VectorXd(0), Eigen::VectorXd(0),
                                       0.5, &constraint_residue, &cost);
    EXPECT_EQ(dut.s_size(), 0);
  }
  EXPECT_EQ(al_val, 0);
  EXPECT_EQ(constraint_residue.rows(), 0);
  Eigen::VectorXd x_lo_expected, x_up_expected;
  AggregateBoundingBoxConstraints(dut.prog(), &x_lo_expected, &x_up_expected);
  EXPECT_TRUE(CompareMatrices(dut.x_lo(), x_lo_expected));
  EXPECT_TRUE(CompareMatrices(dut.x_up(), x_up_expected));
}

GTEST_TEST(EvalAugmentedLagrangian, BoundingBoxConstraint) {
  // Test with bounding box constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  auto constraint1 = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d(1, 3, -kInf, 0), Eigen::Vector4d(2, kInf, -1, 5), x);
  auto constraint2 = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d(2, 1, -kInf, 1), Eigen::Vector4d(5, kInf, -1, 3), x);

  const double tol = 1E-10;
  CheckAugmentedLagrangianNonsmoothBoundingBoxConstraint(
      prog, Eigen::Vector4d(1, 3, 5, 2),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.3, 1.5, 2, 1).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianNonsmoothBoundingBoxConstraint(
      prog, math::InitializeAutoDiff(Eigen::Vector4d(1, 3, 5, 2)),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.3, 1.5, 2, 1).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianSmoothBoundingBoxConstraint(
      prog, Eigen::Vector4d(1, 3, 5, 2), Eigen::Vector4d(2, 1, -2, 3),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.3, 1.5, 2, 1).finished(), 0.5,
      tol);
  CheckAugmentedLagrangianSmoothBoundingBoxConstraint(
      prog, math::InitializeAutoDiff(Eigen::Vector4d(1, 3, 5, 2)),
      math::InitializeAutoDiff(Eigen::Vector4d(2, 1, -2, 3)),
      (Eigen::Matrix<double, 5, 1>() << 0.5, 0.3, 1.5, 2, 1).finished(), 0.5,
      tol);

  // Test include_x_bounds = false, the augmented lagrangian is 0.
  const AugmentedLagrangianNonsmooth dut_nonsmooth(&prog, false);
  CheckBoundingBoxConstraintEmpty(dut_nonsmooth);
  const AugmentedLagrangianSmooth dut_smooth(&prog, false);
  CheckBoundingBoxConstraintEmpty(dut_smooth);
}
}  // namespace
}  // namespace solvers
}  // namespace drake

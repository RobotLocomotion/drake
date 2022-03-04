#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/scs_solver.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using drake::solvers::Binding;
using drake::solvers::LorentzConeConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::ScsSolver;
using drake::solvers::SolverOptions;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

constexpr double kTolerance = 1.0e-8;

// This method solves the projection in the norm defined by R:
//   min 1/2(γ−y)ᵀ⋅R⋅(γ−y)
//   s.t. γ ∈ ℱ
// where ℱ = {x ∈ ℝ³ | sqrt(x₀²+x₁²) ≤ μx₂} is the friction cone defined by mu.
// We use ScsSolver for conic programs to solve it.
// R is a positive diagonal matrix. Here only the non-zero entries of the
// diagonal need to be supplied as a Vector3d.
Vector3d SolveProjectionWithScs(double mu, const Vector3d& R,
                                const Vector3d& y) {
  MathematicalProgram prog;
  Eigen::Matrix<symbolic::Variable, 3, 1> gamma =
      prog.NewContinuousVariables<3>();
  // Add cost ℓ(γ) = 1/2(γ−y)ᵀ⋅R⋅(γ−y)
  const Matrix3d Q = R.asDiagonal();
  prog.AddQuadraticErrorCost(Q, y, gamma);

  // Add friction cone constraint sqrt(γ₀²+γ₁²) ≤ μγ₂.
  //
  // Per documentation in LorentzConeConstraint, a vector x satisfies the
  // Lorentz cone constraint if:
  //   sqrt(z₁²+z₂²) ≤ z₀,
  //   with z = A⋅x+b.
  // Therefore, making x=γ, b=0 and
  //     |0 0 μ|
  // A = |1 0 0|
  //     |0 1 0|
  // γ satisfies the Lorentz cone constraint if sqrt(γ₀²+γ₁²) ≤ μγ₂.
  const Matrix3d A =
      (Matrix3d() << 0., 0., mu, 1., 0., 0., 0., 1., 0.).finished();
  const Vector3d b = Vector3d::Zero();
  auto cone_constraint = std::make_shared<LorentzConeConstraint>(
      A, b, LorentzConeConstraint::EvalType::kConvexSmooth);  // kConvexSmooth
  Binding<LorentzConeConstraint> binding(cone_constraint, gamma);
  prog.AddConstraint(binding);

  // Now setup the SCS solver.
  ScsSolver solver;
  SolverOptions options;
  // Mathematical program sets these tolerances to 1.0e-5 by default. To compare
  // against analytical exact solutions those tolerances are too loose. We
  // tighten them.
  options.SetOption(ScsSolver::id(), "eps_abs", kTolerance);
  options.SetOption(ScsSolver::id(), "eps_rel", kTolerance);
  MathematicalProgramResult result;
  solver.Solve(prog, y, options, &result);
  DRAKE_DEMAND(result.is_success());
  return result.GetSolution();
}

// This method is used to validate SapFrictionConeConstraint::Project().
// We use MathematicalProgram to obtain numerical values that we use to compare
// againthe analytical projection implemented by SapFrictionConeConstraint.
// To validate the analytical gradients of the projection, we use automatic
// differentiation.
void ValidateProjection(double mu, const Vector3d& R, const Vector3d& y) {
  const double bad_number = std::numeric_limits<double>::infinity();
  SapFrictionConeConstraint<AutoDiffXd>::Parameters p{
      mu, bad_number, bad_number, bad_number, bad_number};
  const int clique = 0;
  const AutoDiffXd phi0 = bad_number;
  const Matrix3<AutoDiffXd> J = Matrix3<AutoDiffXd>::Constant(bad_number);
  SapFrictionConeConstraint<AutoDiffXd> c(clique, J, phi0, p);
  Vector3<AutoDiffXd> y_ad = drake::math::InitializeAutoDiff(y);
  Vector3<AutoDiffXd> R_ad(R);
  Vector3<AutoDiffXd> gamma_ad;
  MatrixX<AutoDiffXd> dPdy_ad;
  c.Project(y_ad, R_ad, &gamma_ad, &dPdy_ad);
  const Vector3d gamma = math::ExtractValue(gamma_ad);

  // We first validate the result of the projection γ = P(y).
  const Vector3d gamma_numerical = SolveProjectionWithScs(mu, R, y);
  PRINT_VAR(gamma.transpose());
  PRINT_VAR(gamma_numerical.transpose());
  PRINT_VAR((gamma - gamma_numerical).norm());
  EXPECT_TRUE(CompareMatrices(gamma, gamma_numerical, kTolerance,
                              MatrixCompareType::relative));

  // We now verify gradients using automatic differentiation.
  const Matrix3d dPdy = math::ExtractValue(dPdy_ad);
  // N.B. We supply num_derivatives so that when the gradient is zero,
  // ExtractGradient() does not return a zero sized matrix but Matrix3d::Zero().
  const int num_derivatives = 3;
  const Matrix3d gamma_ad_gradient =
      math::ExtractGradient(gamma_ad, num_derivatives);
  PRINT_VARn(dPdy);
  PRINT_VARn(gamma_ad_gradient);
  EXPECT_TRUE(CompareMatrices(dPdy, gamma_ad_gradient,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Region I corresponds to the friction cone, see [Castro et al., 2021].
// Phisically this is the stiction region.
GTEST_TEST(FrictionConeConstraint, RegionI) {
  const double mu = 0.5;
  const Vector3d R(0.1, 0.1, 1.0);
  const Vector3d y(0.4, 0, 1.0);
  ValidateProjection(mu, R, y);
}

// Region II corresponds to ℝ³ minus Regions I and II, see [Castro et al.,
// 2021]. Physically this is the sliding region.
GTEST_TEST(FrictionConeConstraint, RegionII) {
  const double mu = 0.5;
  const Vector3d R(0.1, 0.1, 1.0);
  const Vector3d y(1.0, 0, 1.0);
  ValidateProjection(mu, R, y);
}

// Region III corresponds to the polar cone, see [Castro et al., 2021].
// Phisically this is the no contact region, i.e. gamma = 0.
GTEST_TEST(FrictionConeConstraint, RegionIII) {
  const double mu = 0.5;
  const Vector3d R(0.1, 0.1, 1.0);
  const Vector3d y(0.5, 0, -0.2);
  ValidateProjection(mu, R, y);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

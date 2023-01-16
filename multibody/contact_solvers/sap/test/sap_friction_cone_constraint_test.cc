#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/scs_solver.h"

using drake::solvers::Binding;
using drake::solvers::LorentzConeConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::ScsSolver;
using drake::solvers::SolverOptions;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace kcov339_avoidance_magic {
namespace {

// These Jacobian matrices have arbitrary values for testing. We specify the
// size of the matrix in the name, e.g. J32 is of size 3x2.
// clang-format off
const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();

const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

GTEST_TEST(SapFrictionConeConstraint, SingleCliqueConstraint) {
  const double mu = 0.5;
  const double stiffness = 1.0e5;
  const double dissipation_time_scale = 0.01;
  const double beta = 0.1;
  const double sigma = 1.0e-4;
  const int clique = 12;
  const double phi0 = -2.5e-3;
  SapFrictionConeConstraint<double>::Parameters parameters{
      mu, stiffness, dissipation_time_scale, beta, sigma};
  SapFrictionConeConstraint<double> c(clique, J32, phi0, parameters);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), clique);
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.constraint_function(), Vector3d(0., 0., phi0));
  EXPECT_EQ(c.first_clique_jacobian(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
  EXPECT_EQ(c.mu(), mu);
  EXPECT_EQ(c.parameters().mu, mu);
  EXPECT_EQ(c.parameters().stiffness, stiffness);
  EXPECT_EQ(c.parameters().dissipation_time_scale, dissipation_time_scale);
  EXPECT_EQ(c.parameters().beta, beta);
  EXPECT_EQ(c.parameters().sigma, sigma);
}

GTEST_TEST(SapFrictionConeConstraint, TwoCliquesConstraint) {
  const double mu = 0.5;
  const double stiffness = 1.0e5;
  const double dissipation_time_scale = 0.01;
  const double beta = 0.1;
  const double sigma = 1.0e-4;
  const int clique0 = 12;
  const int clique1 = 13;
  const double phi0 = -2.5e-3;
  SapFrictionConeConstraint<double>::Parameters parameters{
      mu, stiffness, dissipation_time_scale, beta, sigma};
  SapFrictionConeConstraint<double> c(clique0, clique1, J32, J34, phi0,
                                      parameters);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), clique0);
  EXPECT_EQ(c.second_clique(), clique1);
  EXPECT_EQ(c.constraint_function(), Vector3d(0., 0., phi0));
  EXPECT_EQ(c.first_clique_jacobian(), J32);
  EXPECT_EQ(c.second_clique_jacobian(), J34);
  EXPECT_EQ(c.mu(), mu);
  EXPECT_EQ(c.parameters().mu, mu);
  EXPECT_EQ(c.parameters().stiffness, stiffness);
  EXPECT_EQ(c.parameters().dissipation_time_scale, dissipation_time_scale);
  EXPECT_EQ(c.parameters().beta, beta);
  EXPECT_EQ(c.parameters().sigma, sigma);
}

GTEST_TEST(SapFrictionConeConstraint, CalcBias) {
  // We set parameters that we expect do not participate in the computation to a
  // bad number. If they somehow participate in the computation we'd find out
  // quickly.
  const double bad_number = std::numeric_limits<double>::infinity();
  const double mu = bad_number;
  const double stiffness = bad_number;
  const double dissipation_time_scale = 0.01;
  const double beta = bad_number;
  const double sigma = bad_number;
  const int clique = 12;
  const double phi0 = -2.5e-3;
  const Matrix3d J = Matrix3d::Constant(bad_number);
  SapFrictionConeConstraint<double>::Parameters parameters{
      mu, stiffness, dissipation_time_scale, beta, sigma};
  SapFrictionConeConstraint<double> c(clique, J, phi0, parameters);

  const double time_step = 5e-3;
  const double delassus_approximation = NAN;  // Does not participate.
  const Vector3d vhat = c.CalcBiasTerm(time_step, delassus_approximation);
  const Vector3d vhat_expected =
      -c.constraint_function() / (time_step + dissipation_time_scale);
  EXPECT_TRUE(CompareMatrices(vhat, vhat_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

GTEST_TEST(SapFrictionConeConstraint, CalcRegularization) {
  // We set parameters that we expect do not participate in the computation to a
  // bad number. If they somehow participate in the computation we'd find out
  // quickly.
  const double bad_number = std::numeric_limits<double>::infinity();
  const double mu = 0.5;
  const double stiffness = 1.0e5;
  const double dissipation_time_scale = 0.01;
  const double beta = 0.1;
  const double sigma = 1.0e-4;
  const int clique = 12;
  const double phi0 = -2.5e-3;
  const Matrix3d J = Matrix3d::Constant(bad_number);
  SapFrictionConeConstraint<double>::Parameters parameters{
      mu, stiffness, dissipation_time_scale, beta, sigma};
  SapFrictionConeConstraint<double> c(clique, J, phi0, parameters);

  const double time_step = 5e-3;
  const double delassus_approximation = NAN;  // Does not participate.
  const Vector3d R =
      c.CalcDiagonalRegularization(time_step, delassus_approximation);

  const double Rt = sigma * delassus_approximation;
  const double Rn = std::max(
      beta * beta / (4 * M_PI * M_PI) * delassus_approximation,
      1. / (time_step * (time_step + dissipation_time_scale) * stiffness));

  const Vector3d R_expected(Rt, Rt, Rn);
  EXPECT_TRUE(CompareMatrices(R, R_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

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
  auto cone_constraint = std::make_shared<LorentzConeConstraint>(A, b);
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
// against the analytical projection implemented by SapFrictionConeConstraint.
// To validate the analytical gradients of the projection, we use automatic
// differentiation.
void ValidateProjection(double mu, const Vector3d& R, const Vector3d& y) {
  // We set parameters that we expect do not participate in the computation to a
  // bad number. If they somehow participate in the computation we'd find out
  // quickly.
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
  EXPECT_TRUE(CompareMatrices(gamma, gamma_numerical, 10.0 * kTolerance,
                              MatrixCompareType::relative));

  // We now verify gradients using automatic differentiation.
  const Matrix3d dPdy = math::ExtractValue(dPdy_ad);
  // N.B. We supply num_derivatives so that when the gradient is zero,
  // ExtractGradient() does not return a zero sized matrix but Matrix3d::Zero().
  const int num_derivatives = 3;
  const Matrix3d gamma_ad_gradient =
      math::ExtractGradient(gamma_ad, num_derivatives);
  EXPECT_TRUE(CompareMatrices(dPdy, gamma_ad_gradient,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Region I corresponds to the friction cone, see [Castro et al., 2021].
// Physically this is the stiction region.
GTEST_TEST(SapFrictionConeConstraint, RegionI) {
  // Below we use an arbitrary set of values so that y is in Region I
  // (stiction).
  {
    const double mu = 0.5;
    const Vector3d R(0.1, 0.1, 1.5);
    const Vector3d y(0.4, 0, 1.0);
    ValidateProjection(mu, R, y);
  }
  {
    const double mu = 1.5;
    const Vector3d R(0.01, 0.01, 2.0);
    const Vector3d y(1.0, -1.0, 2.0);
    ValidateProjection(mu, R, y);
  }
  {
    const double mu = 0.3;
    const Vector3d R(0.2, 0.2, 1.2);
    const Vector3d y(-0.1, 0.01, 1.0);
    ValidateProjection(mu, R, y);
  }
}

// Region II corresponds to ℝ³ minus Regions I and II, see [Castro et al.,
// 2021]. Physically this is the sliding region.
GTEST_TEST(SapFrictionConeConstraint, RegionII) {
  // Below we use an arbitrary set of values so that y is in Region II
  // (sliding).
  {
    const double mu = 0.5;
    const Vector3d R(0.1, 0.1, 1.5);
    const Vector3d y(1.0, 0, 1.0);
    ValidateProjection(mu, R, y);
  }
  {
    const double mu = 1.5;
    const Vector3d R(0.01, 0.01, 2.0);
    const Vector3d y(4.0, 2.5, 2.0);
    ValidateProjection(mu, R, y);
  }
  {
    const double mu = 0.3;
    const Vector3d R(0.2, 0.2, 1.2);
    // N.B. This is an interesting case. Since the projection is convex, not all
    // negative values of y(2) lead to gamma = 0. However, gamma(2) is always
    // positive.
    const Vector3d y(-0.5, 1.8, -0.01);
    ValidateProjection(mu, R, y);
  }
}

// Region III corresponds to the polar cone, see [Castro et al., 2021].
// Physically this is the no contact region, i.e. gamma = 0.
GTEST_TEST(SapFrictionConeConstraint, RegionIII) {
  // Below we use an arbitrary set of values so that y is in Region III (no
  // contact).
  {
    const double mu = 0.5;
    const Vector3d R(0.1, 0.1, 1.5);
    const Vector3d y(0.5, 0, -0.2);
    ValidateProjection(mu, R, y);
  }
  {
    const double mu = 1.5;
    const Vector3d R(0.01, 0.01, 2.0);
    const Vector3d y(4.0, 2.5, -1.3);
    ValidateProjection(mu, R, y);
  }
  {
    const double mu = 0.3;
    const Vector3d R(0.2, 0.2, 1.2);
    const Vector3d y(-0.5, 1.8, -5.0);
    ValidateProjection(mu, R, y);
  }
}

GTEST_TEST(SapFrictionConeConstraint, SingleCliqueConstraintClone) {
  const double mu = 0.5;
  const double stiffness = 1.0e5;
  const double dissipation_time_scale = 0.01;
  const double beta = 0.1;
  const double sigma = 1.0e-4;
  const int clique = 12;
  const double phi0 = -2.5e-3;
  SapFrictionConeConstraint<double>::Parameters parameters{
      mu, stiffness, dissipation_time_scale, beta, sigma};
  SapFrictionConeConstraint<double> c(clique, J32, phi0, parameters);
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone =
      dynamic_pointer_cast<SapFrictionConeConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), clique);
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->constraint_function(), Vector3d(0., 0., phi0));
  EXPECT_EQ(clone->first_clique_jacobian(), J32);
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
  EXPECT_EQ(clone->mu(), mu);
  EXPECT_EQ(clone->parameters().mu, mu);
  EXPECT_EQ(clone->parameters().stiffness, stiffness);
  EXPECT_EQ(clone->parameters().dissipation_time_scale, dissipation_time_scale);
  EXPECT_EQ(clone->parameters().beta, beta);
  EXPECT_EQ(clone->parameters().sigma, sigma);
}

GTEST_TEST(SapFrictionConeConstraint, TwoCliquesConstraintClone) {
  const double mu = 0.5;
  const double stiffness = 1.0e5;
  const double dissipation_time_scale = 0.01;
  const double beta = 0.1;
  const double sigma = 1.0e-4;
  const int clique0 = 12;
  const int clique1 = 13;
  const double phi0 = -2.5e-3;
  SapFrictionConeConstraint<double>::Parameters parameters{
      mu, stiffness, dissipation_time_scale, beta, sigma};
  SapFrictionConeConstraint<double> c(clique0, clique1, J32, J34, phi0,
                                      parameters);
  auto clone =
      dynamic_pointer_cast<SapFrictionConeConstraint<double>>(c.Clone());
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 2);
  EXPECT_EQ(clone->first_clique(), clique0);
  EXPECT_EQ(clone->second_clique(), clique1);
  EXPECT_EQ(clone->constraint_function(), Vector3d(0., 0., phi0));
  EXPECT_EQ(clone->first_clique_jacobian(), J32);
  EXPECT_EQ(clone->second_clique_jacobian(), J34);
  EXPECT_EQ(clone->mu(), mu);
  EXPECT_EQ(clone->parameters().mu, mu);
  EXPECT_EQ(clone->parameters().stiffness, stiffness);
  EXPECT_EQ(clone->parameters().dissipation_time_scale, dissipation_time_scale);
  EXPECT_EQ(clone->parameters().beta, beta);
  EXPECT_EQ(clone->parameters().sigma, sigma);
}

}  // namespace
}  // namespace kcov339_avoidance_magic
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

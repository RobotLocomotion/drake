#include "drake/multibody/contact_solvers/sap/sap_pd_controller_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using Parameters = SapPdControllerConstraint<double>::Parameters;
using Configuration = SapPdControllerConstraint<double>::Configuration;

namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

template <typename T = double>
typename SapPdControllerConstraint<T>::Parameters MakeArbitraryParameters() {
  const T Kp = 1.1;
  const T Kd = 1.2;
  const T effort_limit = 1.3;
  return
      typename SapPdControllerConstraint<T>::Parameters{Kp, Kd, effort_limit};
}

template <typename T = double>
typename SapPdControllerConstraint<T>::Configuration
MakeArbitraryConfiguration() {
  const int clique = 4;
  const int clique_dof = 2;
  const int clique_nv = 5;
  const T q0 = 0.1;
  const T qd = 0.2;
  const T vd = 0.3;
  const T u0 = 0.4;
  return typename SapPdControllerConstraint<T>::Configuration{
      clique, clique_dof, clique_nv, q0, qd, vd, u0};
}

void ExpectEqual(const SapPdControllerConstraint<double>& c1,
                 const SapPdControllerConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.parameters(), c2.parameters());
  EXPECT_EQ(c1.configuration(), c2.configuration());
}

/* SapPdControllerConstraint models (without considering effort limits) the
 actuation u at the next time step as:
   u = -Kp⋅(q − qd) - Kd⋅(v − vd) + u₀
 SAP uses a first order approximation of q:
   q = q₀ + δt⋅v
 Therefore, for a given value of actuation u we compute the velocity needed to
 achieve that effort, as:
   v = (u₀ − u + Kp⋅(qd - q₀) + Kd⋅vd) / (δt⋅Kp + Kd)
 The actuation required to achieve this value of velocity might exceed effort
 limits. */
double CalcVelocityFromActuationValue(const Configuration& k,
                                      const Parameters& p, double dt,
                                      double u) {
  const double q0 = k.q0;
  const double qd = k.qd;
  const double vd = k.vd;
  const double u0 = k.u0;
  const double Kp = p.Kp();
  const double Kd = p.Kd();
  const double v = (u0 - u + Kp * (qd - q0) + Kd * vd) / (dt * Kp + Kd);
  return v;
}

GTEST_TEST(SapPdControllerConstraint, Construction) {
  Parameters parameters = MakeArbitraryParameters();
  Configuration configuration = MakeArbitraryConfiguration();
  SapPdControllerConstraint<double> c(configuration, parameters);

  EXPECT_EQ(c.num_objects(), 0);
  EXPECT_EQ(c.num_constraint_equations(), 1);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), configuration.clique);
  EXPECT_EQ(c.num_velocities(0), configuration.clique_nv);

  // Configuration at construction.
  EXPECT_EQ(c.configuration(), configuration);

  // Parameters at construction.
  EXPECT_EQ(c.parameters(), parameters);

  // Always a single clique.
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);

  // The Jacobian should contain only a single one at the specified dof and zero
  // elsewhere.
  const MatrixX<double> J = c.first_clique_jacobian().MakeDenseMatrix();
  const MatrixX<double> J_expected = RowVectorX<double>::Unit(
      configuration.clique_nv, configuration.clique_dof);
  EXPECT_EQ(J, J_expected);
}

GTEST_TEST(SapPdControllerConstraint, CalcImpulse) {
  const double dt = 0.015;
  const Parameters p = MakeArbitraryParameters();
  const Configuration k = MakeArbitraryConfiguration();
  const SapPdControllerConstraint<double> c(k, p);
  // Zero Delassus, and thus we know we are not in the "near-rigid" regime.
  const VectorXd w = Vector1d::Zero();
  auto data = c.MakeData(dt, w);

  // Helper to compute force using CalcImpulse() for a value of velocity
  // computed from u using CalcVelocityFromActuationValue().
  auto calc_force = [&](double u) -> double {
    const double v = CalcVelocityFromActuationValue(k, p, dt, u);
    const VectorXd vc = Vector1d::Constant(v);
    c.CalcData(vc, data.get());
    VectorXd gamma(1);
    c.CalcImpulse(*data, &gamma);
    return gamma(0) / dt;
  };

  const double e = p.effort_limit();

  // Value within effort limits.
  {
    const double u = p.effort_limit() * 0.2;
    const double f = calc_force(u);
    EXPECT_NEAR(f, u, kEps);
  }

  // Value above effort limit.
  {
    const double u = e * 1.2;
    const double f = calc_force(u);
    EXPECT_EQ(f, e);
  }

  // Value below effort limit.
  {
    const double u = -e * 1.2;
    const double f = calc_force(u);
    EXPECT_EQ(f, -e);
  }
}

// Unit test SapPdControllerConstraint::MakeData() outside the near-rigid
// regime. Gains, time step and Delassus operator values are set so that the
// constraint is created in a configuration outside the near-rigid regime.
GTEST_TEST(SapPdControllerConstraint, MakeDataOutsideTheNearRigidRegime) {
  const double dt = 0.015;
  const Parameters p = MakeArbitraryParameters();
  // The configuration is irrelevant for this test.
  const Configuration k = MakeArbitraryConfiguration();
  const SapPdControllerConstraint<double> c(k, p);
  // Arbitrary value for the Delassus operator.
  const VectorXd w = Vector1d::Constant(1.6);

  // Verify that gains were not clamped.
  auto abstract_data = c.MakeData(dt, w);
  const auto& data =
      abstract_data->get_value<SapPdControllerConstraintData<double>>();
  EXPECT_EQ(data.Kp_eff(), p.Kp());
  EXPECT_EQ(data.Kd_eff(), p.Kd());
  EXPECT_EQ(data.time_step(), dt);
}

// Unit test SapPdControllerConstraint::MakeData() in the near-rigid regime.
// Gains are set to high values so that the constraint is created in a
// configuration within the near-rigid regime.
GTEST_TEST(SapPdControllerConstraint, MakeDataNearRigidRegime) {
  // This value of near-rigid parameter is the one internally used by
  // SapPdControllerConstraint and they must remain in sync.
  constexpr double kBeta = 0.1;

  const double dt = 0.015;
  const double large_Kp = 1e8;
  const double tau = 0.13;
  const double large_Kd = tau * large_Kp;
  const double effort_limit = 0.3;
  const Parameters p{large_Kp, large_Kd, effort_limit};
  // The configuration is irrelevant for this test.
  const Configuration k = MakeArbitraryConfiguration();
  const SapPdControllerConstraint<double> c(k, p);
  // Arbitrary value for the Delassus operator.
  const VectorXd w = Vector1d::Constant(1.6);

  const double Rnr = kBeta * kBeta / (4.0 * M_PI * M_PI) * w(0);
  // Expected near-rigid parameters. Time scale tau is kept constant.
  const double Kp_nr = 1.0 / Rnr / (dt * (dt + tau));
  const double Kd_nr = tau * Kp_nr;

  // Verify that gains were clamped to be in the near-rigid regime.
  auto abstract_data = c.MakeData(dt, w);
  const auto& data =
      abstract_data->get_value<SapPdControllerConstraintData<double>>();
  EXPECT_NEAR(data.Kp_eff(), Kp_nr, kEps * large_Kp);
  EXPECT_NEAR(data.Kd_eff(), Kd_nr, kEps * large_Kd);
  EXPECT_EQ(data.time_step(), dt);  // not affected.
}

// Verify the effective gains within the near-rigid regime when the user sets
// the position gain Kp to zero. In particular, the effective position gain
// should remain zero to respect the user's desire to do velocity control only.
GTEST_TEST(SapPdControllerConstraint, MakeDataNearRigidRegimeWithZeroKp) {
  // This value of near-rigid parameter is the one internally used by
  // SapPdControllerConstraint and they must remain in sync.
  constexpr double kBeta = 0.1;

  const double dt = 0.015;
  const double Kp = 0.0;
  const double large_Kd = 1.0e8;
  const double effort_limit = 0.3;
  const Parameters p{Kp, large_Kd, effort_limit};
  // The configuration is irrelevant for this test.
  const Configuration k = MakeArbitraryConfiguration();
  const SapPdControllerConstraint<double> c(k, p);
  // Arbitrary value for the Delassus operator.
  const VectorXd w = Vector1d::Constant(1.6);

  const double Rnr = kBeta * kBeta / (4.0 * M_PI * M_PI) * w(0);
  // Expected near-rigid parameters. The user wants velocity control, and we
  // should respect that.
  const double Kd_nr = 1.0 / (dt * Rnr);

  // Verify that gains were clamped to be in the near-rigid regime.
  auto abstract_data = c.MakeData(dt, w);
  const auto& data =
      abstract_data->get_value<SapPdControllerConstraintData<double>>();
  EXPECT_EQ(data.Kp_eff(), 0.0);
  EXPECT_NEAR(data.Kd_eff(), Kd_nr, kEps);
  EXPECT_EQ(data.time_step(), dt);  // not affected.
}

// Verify the effective gains within the near-rigid regime when the user sets
// the derivative gain Kd to zero. In particular, the effective derivate gain
// should remain zero to respect the user's desire to do position control only.
GTEST_TEST(SapPdControllerConstraint, MakeDataNearRigidRegimeWithZeroKd) {
  // This value of near-rigid parameter is the one internally used by
  // SapPdControllerConstraint and they must remain in sync.
  constexpr double kBeta = 0.1;

  const double dt = 0.015;
  const double large_Kp = 1e8;
  const double Kd = 0.0;
  const double effort_limit = 0.3;
  const Parameters p{large_Kp, Kd, effort_limit};
  // The configuration is irrelevant for this test.
  const Configuration k = MakeArbitraryConfiguration();
  const SapPdControllerConstraint<double> c(k, p);
  // Arbitrary value for the Delassus operator.
  const VectorXd w = Vector1d::Constant(1.6);

  const double Rnr = kBeta * kBeta / (4.0 * M_PI * M_PI) * w(0);
  // Expected near-rigid parameters. The user wants position control, and we
  // should respect that.
  const double Kp_nr = 1.0 / (dt * dt * Rnr);

  // Verify that gains were clamped to be in the near-rigid regime.
  auto abstract_data = c.MakeData(dt, w);
  const auto& data =
      abstract_data->get_value<SapPdControllerConstraintData<double>>();
  EXPECT_NEAR(data.Kp_eff(), Kp_nr, kEps);
  EXPECT_EQ(data.Kd_eff(), 0.0);
  EXPECT_EQ(data.time_step(), dt);  // not affected.
}

// This method validates analytical gradients implemented by
// SapPdControllerConstraint using automatic differentiation.
void ValidateGradients(double v) {
  const Vector1d vc(v);  // Constraint velocity vector.
  // Arbitrary kinematic and parameter values.
  const SapPdControllerConstraint<AutoDiffXd>::Parameters p =
      MakeArbitraryParameters<AutoDiffXd>();
  const SapPdControllerConstraint<AutoDiffXd>::Configuration k =
      MakeArbitraryConfiguration<AutoDiffXd>();

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapPdControllerConstraint<AutoDiffXd> c(k, p);

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

GTEST_TEST(SapPdControllerConstraint, Gradients) {
  const double dt = 0.015;
  const Parameters p = MakeArbitraryParameters();
  const Configuration k = MakeArbitraryConfiguration();
  // Arbitrary set of velocity values.
  {
    // Value within effort limits.
    const double u = p.effort_limit() * 0.2;
    const double v = CalcVelocityFromActuationValue(k, p, dt, u);
    ValidateGradients(v);
  }
  {
    // Value above effort limit.
    const double u = p.effort_limit() * 1.2;
    const double v = CalcVelocityFromActuationValue(k, p, dt, u);
    ValidateGradients(v);
  }
  {
    // Value below effort limit.
    const double u = -p.effort_limit() * 1.2;
    const double v = CalcVelocityFromActuationValue(k, p, dt, u);
    ValidateGradients(v);
  }
}

GTEST_TEST(SapPdControllerConstraint, Clone) {
  Parameters parameters = MakeArbitraryParameters();
  Configuration configuration = MakeArbitraryConfiguration();
  SapPdControllerConstraint<double> c(configuration, parameters);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone =
      dynamic_pointer_cast<SapPdControllerConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapPdControllerConstraint<AutoDiffXd> c_ad(
      MakeArbitraryConfiguration<AutoDiffXd>(),
      MakeArbitraryParameters<AutoDiffXd>());
  auto clone_from_ad =
      dynamic_pointer_cast<SapPdControllerConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapPdControllerConstraint, AccumulateGeneralizedImpulses) {
  Parameters parameters = MakeArbitraryParameters();
  Configuration configuration = MakeArbitraryConfiguration();
  SapPdControllerConstraint<double> c(configuration, parameters);

  // Arbitrary value of the impulse.
  const Vector1d gamma(1.2345);

  // Expected generalized impulse.
  VectorXd tau_clique = VectorXd::Zero(configuration.clique_nv);
  tau_clique(configuration.clique_dof) = gamma(0);

  // Arbitrary value on which to start accumulating from.
  const VectorXd tau_0 =
      VectorXd::LinSpaced(configuration.clique_nv, 1, configuration.clique_nv);

  VectorXd tau_accumulated = tau_0;  // Initialize to non-zero value.
  const VectorXd tau_expected = tau_0 + tau_clique;
  c.AccumulateGeneralizedImpulses(0, gamma, &tau_accumulated);
  EXPECT_TRUE(CompareMatrices(tau_accumulated, tau_expected, kEps,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

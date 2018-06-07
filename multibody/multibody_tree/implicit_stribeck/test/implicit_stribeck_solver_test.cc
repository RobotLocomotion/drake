#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {
namespace {

/* Top view of the pizza saver:
  ^ y               C
  |                 ◯
  |                /\
  ----> x         /  \
               b /    \ a
                /      \
               /        \
              ◯----------◯
              A     c    B
It is modeled as an equilateral triangle with a contact point at each of the
legs. The total mass of the pizza saver is m and its rotational inertia about
the triangle's barycenter is I.
If h is the height of the triangle from any of its sides, the distance from
any point to the triangle's center is 2h/3. The height h relates to the size
a of a side by h = 3/2/sqrt(3) a.
The generalized positions vector for this case is q = [x, y, theta], with
theta = 0 for the triangle in the configuration shown in the schematic. */
class PizzaSaver : public ::testing::Test {
 public:
  void SetUp() override {
    // Now we'll set up each term in the equation:
    //   Mv̇ = τ + Dᵀ⋅fₜ
    // where τ =[Fx, Fy, Mz] contains the external force in x, the external
    // force in y and the external moment about z (out of plane).
    M_ << m_,  0,  0,
        0, m_,  0,
        0,  0,  I_;
  }

  MatrixX<double> ComputeTangentialJacobian(double theta) {
    MatrixX<double> D(2 * nc_, nv_);
    const double c = cos(theta);
    const double s = sin(theta);

    // 2D rotation matrix of the body frame B in the world frame W.
    Matrix2<double> R_WB;
    R_WB <<  c, s,
        -s, c;

    // Position of each contact point in the body frame B.
    const Vector2<double> p_BoA(-sqrt(3) / 2.0, -0.5);
    const Vector2<double> p_BoB(sqrt(3) / 2.0, -0.5);
    const Vector2<double> p_BoC(0.0, 1.0);

    // Position of each contact point in the world frame W.
    const Vector2<double> p_BoA_W = R_WB * p_BoA;
    const Vector2<double> p_BoB_W = R_WB * p_BoB;
    const Vector2<double> p_BoC_W = R_WB * p_BoC;

    // Point A
    D.block(0, 0, 2, nv_) << 1, 0, -p_BoA_W.y(),
        0, 1,  p_BoA_W.x();

    // Point B
    D.block(2, 0, 2, nv_) << 1, 0, -p_BoB_W.y(),
        0, 1,  p_BoB_W.x();

    // Point C
    D.block(4, 0, 2, nv_) << 1, 0, -p_BoC_W.y(),
        0, 1,  p_BoC_W.x();

    return D;
  }

  void SetProblem(const Vector3<double>& v0, const Vector3<double>& tau,
                  double mu, double theta, double dt) {
    // Next time step generalized momentum if there are no friction forces.
    p_star_ = M_ * v0 + dt * tau;

    // Normal forces. Assume they are equally distributed.
    fn_ = m_ * g_ / 3.0 * Vector3<double>::Ones();

    // All contact points have the same friction for this case.
    mu_ = mu * Vector3<double>::Ones();

    D_ = ComputeTangentialJacobian(theta);

    solver_.SetProblemData(&M_, &D_, &p_star_, &fn_, &mu_);
  }

 protected:
  // Problem parameters.
  const double m_{1.0};   // Mass of the pizza saver.
  const double R_{1.0};   // Distance from COM to any contact point.
  const double g_{10.0};  // Acceleration of gravity.
  // The radius of the circumscribed circle R is the distance from each
  // contact point to the triangle's center.
  // If we model the pizza saver as three point masses m/3 at each contact
  // point, the moment of inertia is I = 3 * (m/3 R²):
  const double I_{R_ * R_ * m_};  // = 1.0 in this case.

  // Problem sizes.
  const int nv_{3};  // number of generalized velocities.
  const int nc_{3};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> D_{2 * nc_, nv_};

  // The implicit Stribeck solver for this problem.
  ImplicitStribeckSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  Vector3<double> p_star_;  // Generalized momentum.
  Vector3<double> fn_;      // Normal forces at each contact point.
  Vector3<double> mu_;      // Friction coefficient at each contact point.
};

// This test the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (within the Stribeck
// approximation). Otherwise the saver will be sliding.
// For this setup the transition occurs at M_transition = mu * m * g * R = 5.0
TEST_F(PizzaSaver, SmallAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.

  const double mu = 0.5;

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution.
  const double theta = M_PI / 5;

  // External forcing.
  const double Mz = 3.0;  // M_transition = 5.0
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetProblem(v0, tau, mu, theta, dt);

  Parameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  ComputationInfo info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, ComputationInfo::Success);

  VectorX<double> tau_f = solver_.get_generalized_forces();

  const IterationStats& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      /* Dimensionless relative (to the stiction tolerance) tolerance */
      solver_.get_solver_parameters().tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero to machine epsilon.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction should balance the applied Mz. There is though
  // an error introduced by the non-zero stiction tolerance.
  EXPECT_NEAR(tau_f(2), Mz, 5.0e-4);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // The problem has symmetry of revolution. Thus, for any rotation theta,
  // the three tangential velocities should have the same magnitude.
  const double v_slipA = vt.segment<2>(0).norm();
  const double v_slipB = vt.segment<2>(2).norm();
  const double v_slipC = vt.segment<2>(4).norm();
  EXPECT_NEAR(v_slipA, v_slipB, kTolerance);
  EXPECT_NEAR(v_slipA, v_slipC, kTolerance);
  EXPECT_NEAR(v_slipC, v_slipB, kTolerance);

  // For this case where Mz < M_transition, we expect stiction (within the
  // Stribeck's stiction tolerance)
  EXPECT_LT(v_slipA, parameters.stiction_tolerance);
  EXPECT_LT(v_slipB, parameters.stiction_tolerance);
  EXPECT_LT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero within
  // machine epsilon.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);
}

// Exactly the same problem as in PizzaSaver::SmallAppliedMoment but with an
// applied moment Mz = 6.0 > M_transition = 5.0. In this case the pizza saver
// transitions to sliding with a net moment of Mz - M_transition during a
// period (time stepping interval) dt. Therefore we expect a change of angular
// velocity given by Δω = dt⋅(Mz - Mtransition) / I.
TEST_F(PizzaSaver, LargeAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;  // Friction coefficient.

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution.
  const double theta = M_PI / 5;

  // External forcing.
  const double M_transition = 5.0;
  const double Mz = 6.0;
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetProblem(v0, tau, mu, theta, dt);

  Parameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  ComputationInfo info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, ComputationInfo::Success);

  VectorX<double> tau_f = solver_.get_generalized_forces();

  const IterationStats& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      /* Dimensionless relative (to the stiction tolerance) tolerance */
      solver_.get_solver_parameters().tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero to machine epsilon.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);
  // Since we are sliding, the total moment should match M_transition.
  EXPECT_NEAR(tau_f(2), M_transition, 1.0e-13);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // The problem has symmetry of revolution. Thus, for any rotation theta,
  // the three tangential velocities should have the same magnitude.
  const double v_slipA = vt.segment<2>(0).norm();
  const double v_slipB = vt.segment<2>(2).norm();
  const double v_slipC = vt.segment<2>(4).norm();
  EXPECT_NEAR(v_slipA, v_slipB, kTolerance);
  EXPECT_NEAR(v_slipA, v_slipC, kTolerance);
  EXPECT_NEAR(v_slipC, v_slipB, kTolerance);

  // For this case where Mz > M_transition, we expect sliding, so that expected
  // velocities are larger than the stiction tolerance.
  EXPECT_GT(v_slipA, parameters.stiction_tolerance);
  EXPECT_GT(v_slipB, parameters.stiction_tolerance);
  EXPECT_GT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero within
  // machine epsilon.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  const double omega = dt * (Mz - 5.0) / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // Slip velocities should only be due to rotation.
  EXPECT_NEAR(v_slipA, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipB, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipC, R_ * omega, kTolerance);
}

// A test fixture to test DirectionChangeLimiter for a very standard
// configuration of parameters.
class DirectionLimiter : public ::testing::Test {
 protected:
  // Helper to make a 2D rotation matrix.
  static Matrix2<double> Rotation(double theta) {
    return Eigen::Rotation2D<double>(theta).toRotationMatrix();
  }

  // Limiter parameters. See DirectionChangeLimiter for further details.
  const double v_stribeck = 1.0e-4;  // m/s
  const double theta_max = M_PI / 6.0;  // radians.
  const double cos_min = std::cos(theta_max);
  const double tolerance = 0.01;  // Dimensionless. A factor of v_stribeck.
  // Tolerance to perform comparisons close to machine precision.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
};

// Verify results when vt and dvt are exactly zero.
TEST_F(DirectionLimiter, ZeroVandZeroDv) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>::Zero();
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Verify implementation when vt = 0 and the update dvt takes the velocity
// to within the Stribeck circle.
TEST_F(DirectionLimiter, ZeroVtoWithinStictionRegion) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>(-0.5, 0.7) * v_stribeck;
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Perfect stiction (vt = 0) to sliding.
TEST_F(DirectionLimiter, ZeroVtoSlidingRegion) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>(0.3, -0.1);
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  const Vector2<double> vt_alpha_expected = dvt.normalized() * v_stribeck / 2.0;
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Sliding to perfect stiction with vt = 0.
TEST_F(DirectionLimiter, SlidingRegiontoZero) {
  const Vector2<double> vt = Vector2<double>(0.3, -0.1);
  const Vector2<double> dvt = -vt;
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  // DirectionChangeLimiter does not allow changes from outside the stribeck
  // circle (where friction is constant) to exactly zero velociy, since this
  // would imply leaving the solver in a state where gradients are negligible
  // (not strong). The solver can recover from this, but placing the velocity
  // within the Stribeck circle in the direction of the intial v, helps the
  // iterative process even more.
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stribeck / 2.0;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// A vt that lies outside the Stribeck circle lies somewhere within the circle
// after the update v_alpha = v + dv, alpha = 1. Since gradients are strong
// within this region, the limiter allows it.
TEST_F(DirectionLimiter, SlidingRegionToStictionRegion) {
  const Vector2<double> vt = Vector2<double>(1.2, 0.4);
  const Vector2<double> vt_alpha_expected =
      Vector2<double>(-0.3, 0.45) * v_stribeck;
  const Vector2<double> dvt = vt_alpha_expected - vt;
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Similar to ZeroVtoSlidingRegion, a velocity vt within the Stribeck
// region (but not to zero) is updated to a sliding configuration. Since vt
// falls in a region of strong gradients, the limiter allows it.
TEST_F(DirectionLimiter, WithinStictionRegionToSlidingRegion) {
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7) * v_stribeck;
  const Vector2<double> dvt = Vector2<double>(0.9, -0.3);
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Similar to test ZeroVtoSlidingRegion, but vt is not exactly zero
// but negligibly small with norm/v_stribeck < tolerance.
TEST_F(DirectionLimiter, StictionToSliding) {
  const Vector2<double> vt =
      Vector2<double>(-0.5, 0.3) * v_stribeck * tolerance;
  const Vector2<double> dvt(0.3, 0.15);

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  // For this case DirectionChangeLimiter neglects the very small initial vt
  // (since we always have tolerance << 1.0) so that:
  // vα = vt + αΔvt ≈ αΔvt = Δvt/‖Δvt‖⋅vₛ/2.
  // Therefore we expect α = 1 / ‖Δvt‖⋅vₛ/2.
  double alpha_expected = 1.0 / dvt.norm() * v_stribeck / 2.0;

  EXPECT_NEAR(alpha, alpha_expected, kTolerance);
}

// Verifies that the limiter allows negligible changes dvt with alpha = 1.
TEST_F(DirectionLimiter, VerySmallDeltaV) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt =
      Vector2<double>(-0.5, 0.3) * v_stribeck * tolerance;
  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// A very specific scenario when the update vt + dvt crosses zero exactly.
// This is a very common case in 1D-like problems and therefore it does happen
// often.
TEST_F(DirectionLimiter, StraightCrossThroughZero) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt(-0.3, -0.15);  // dvt = -3 * vt.

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  // Since the change crosses zero exactly, we expect
  // v_alpha = v + alpha * dv = v/‖v‖⋅vₛ/2.
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stribeck / 2.0;

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Test a direction change from vt to v1 = vt + dvt that crosses through the
// Stribeck circle. In this case the limiter will find a scalar 0< alpha < 1
// such that v_alpha = vt + alpha * dvt is the closest vector to the origin.
TEST_F(DirectionLimiter, CrossStictionRegionFromTheOutside) {
  // We construct a v_alpha expected to be within the Stribeck circle.
  const Vector2<double> vt_alpha_expected =
      Vector2<double>(0.3, 0.2) * v_stribeck;

  // A unit vector normal to vt_alpha_expected.
  const Vector2<double> vt_normal =
      Vector2<double>(vt_alpha_expected(1), -vt_alpha_expected(0)).normalized();

  // Construct a vt away from the circle in a large magnitude (>>v_stribec) in
  // the direction normal to vt_alpha_expected.
  const Vector2<double> vt = vt_alpha_expected + 0.5 * vt_normal;

  // Construct a v1 away from the circle in a large magnitude (>>v_stribec) in
  // the direction normal to vt_alpha_expected. This time in the opposite
  // direction to that of vt.
  const Vector2<double> v1 = vt_alpha_expected - 0.8 * vt_normal;

  // Velocity change from vt to v1 (this is what the implicit Stribeck iteration
  // would compute).
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  // Verify the result from the limiter.
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the Stribeck circle. In this test, the angle formed by vt and v1 is
// smaller than theta_max and the limiter allows it, i.e. it returns alpha = 1.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion) {
  // an angle smaller that theta_max = M_PI / 6.
  const double theta = M_PI / 8.0;

  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the Stribeck circle. In this test, the angle formed by vt and v1 is
// larger than theta_max and the limiter will limit the change to a
// vt_alpha = vt + alpha * dvt so that vt_alpha forms an angle theta_max with
// vt.
// Internal detail note: the limiter computed two roots with different signs and
// returns the positive root.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion_LargeTheta) {
  // Angle formed by v1 and vt, an angle larger that theta_max = M_PI / 6.
  const double theta1 = M_PI / 3.0;

  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta1) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the Stribeck circle. In this test, the angle formed by vt and v1 is
// MUCH larger than theta_max and the limiter will limit the change to a
// vt_alpha = vt + alpha * dvt so that vt_alpha forms an angle theta_max with
// vt.
// Internal detail note: the limiter computed two positive roots and returns
// the smallest of the two.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion_VeryLargeTheta) {
  // Angle formed by v1 and vt, an angle larger that theta_max = M_PI / 6.
  const double theta1 = 5.0 * M_PI / 6.0;

  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta1) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

// This is a very degenerate case in which the angle formed by vt and dvt
// equals (whithin machine epsilon) to theta_max. It so happens than in this
// case there is a single solution to the quadratic equation solved by the
// limiter (the equation becomes linear).
// Even though this will rarely (or impossibly) happen, we make sure we consider
// it for maximum robustness.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion_SingleSolution) {
  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A dvt forming an angle of theta_max with vt not necessarily having the same
  // magnitude as vt.
  const Vector2<double> dvt = -3.0 * Rotation(theta_max) * vt;

  // Before proceeding with the test, assert that we are in a case where theta1
  // is larger than theta_max.
  const Vector2<double> v1 = vt + dvt;
  double theta1 = std::cos(vt.dot(v1)/vt.norm()/v1.norm());
  ASSERT_GT(theta1, theta_max);

  const double alpha = internal::DirectionChangeLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stribeck, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

}  // namespace
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake


#include "drake/multibody/plant/tamsi_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/multibody/plant/test/tamsi_solver_test_util.h"

namespace drake {
namespace multibody {

// This class has friend access to TamsiSolver so that we can test
// its internals.
class TamsiSolverTester {
 public:
  static MatrixX<double> CalcJacobian(
      const TamsiSolver<double>& solver,
      const Eigen::Ref<const VectorX<double>>& v,
      double dt) {
    const int nv = solver.nv_;

    // Problem data.
    const auto& M = solver.problem_data_aliases_.M();
    const auto& Jn = solver.problem_data_aliases_.Jn();
    const auto& Jt = solver.problem_data_aliases_.Jt();

    // Workspace with size depending on the number of contact points.
    // Note: "auto" below resolves to Eigen::Block.
    auto vn = solver.variable_size_workspace_.mutable_vn();
    auto vt = solver.variable_size_workspace_.mutable_vt();
    auto fn = solver.variable_size_workspace_.mutable_fn();
    auto ft = solver.variable_size_workspace_.mutable_ft();
    auto Gn = solver.variable_size_workspace_.mutable_Gn();
    auto mus = solver.variable_size_workspace_.mutable_mu();
    auto t_hat = solver.variable_size_workspace_.mutable_t_hat();
    auto v_slip = solver.variable_size_workspace_.mutable_v_slip();
    std::vector<Matrix2<double>>& dft_dvt =
        solver.variable_size_workspace_.mutable_dft_dvt();

    // Normal separation velocity.
    vn = Jn * v;

    // Computes friction forces fn and gradients Gn as a function of x, vn,
    // Jn and dt.
    solver.CalcNormalForces(vn, Jn, dt, &fn, &Gn);

    // Tangential velocity.
    vt = Jt * v;

    // Update v_slip, t_hat, mus and ft as a function of vt and fn.
    solver.CalcFrictionForces(vt, fn, &v_slip, &t_hat, &mus, &ft);

    // Compute gradient dft_dvt = ∇ᵥₜfₜ(vₜ) as a function of fn, mus,
    // t_hat and v_slip.
    solver.CalcFrictionForcesGradient(fn, mus, t_hat, v_slip, &dft_dvt);

    // Newton-Raphson Jacobian, J = ∇ᵥR, as a function of M, dft_dvt, Jt, dt.
    MatrixX<double> J(nv, nv);
    solver.CalcJacobian(M, Jn, Jt, Gn, dft_dvt, t_hat, mus, dt, &J);

    return J;
  }

  /// Returns the size of TAMSI's workspace that was last allocated. It is
  /// measured as the number of contact points since the last call to either
  /// SetOneWayCoupledProblemData() or SetTwoWayCoupledProblemData.
  static int get_capacity(const TamsiSolver<double>& solver) {
    return solver.variable_size_workspace_.capacity();
  }
};
namespace {

// A test fixture to test TalsLimiter for a very standard
// configuration of parameters.
class DirectionLimiter : public ::testing::Test {
 protected:
  // Helper to make a 2D rotation matrix.
  static Matrix2<double> Rotation(double theta) {
    return Eigen::Rotation2D<double>(theta).toRotationMatrix();
  }

  // Limiter parameters. See TalsLimiter for further details.
  const double v_stiction = 1.0e-4;  // m/s
  const double theta_max = M_PI / 6.0;  // radians.
  const double cos_min = std::cos(theta_max);
  const double tolerance = 0.01;  // Dimensionless. A factor of v_stiction.
  // Tolerance to perform comparisons close to machine precision.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
};

// Verify results when vt and dvt are exactly zero.
TEST_F(DirectionLimiter, ZeroVandZeroDv) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>::Zero();
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Verify implementation when vt = 0 and the update dvt takes the velocity
// to within the stiction region.
TEST_F(DirectionLimiter, ZeroVtoWithinStictionRegion) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>(-0.5, 0.7) * v_stiction;
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Perfect stiction (vt = 0) to sliding.
TEST_F(DirectionLimiter, ZeroVtoSlidingRegion) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>(0.3, -0.1);
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  const Vector2<double> vt_alpha_expected = dvt.normalized() * v_stiction / 2.0;
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Sliding to perfect stiction with vt = 0.
TEST_F(DirectionLimiter, SlidingRegiontoZero) {
  const Vector2<double> vt = Vector2<double>(0.3, -0.1);
  const Vector2<double> dvt = -vt;
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  // TalsLimiter does not allow changes from outside the stiction region
  // (where friction is constant) to exactly zero velocity, since this
  // would imply leaving the solver in a state where gradients are negligible
  // (not strong). The solver can recover from this, but placing the velocity
  // within the stiction region in the direction of the initial v, helps the
  // iterative process even more.
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stiction / 2.0;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// A vt that lies outside the stiction region lies somewhere within the circle
// after the update v_alpha = v + dv, alpha = 1. Since gradients are strong
// within this region, the limiter allows it.
TEST_F(DirectionLimiter, SlidingRegionToStictionRegion) {
  const Vector2<double> vt = Vector2<double>(1.2, 0.4);
  const Vector2<double> vt_alpha_expected =
      Vector2<double>(-0.3, 0.45) * v_stiction;
  const Vector2<double> dvt = vt_alpha_expected - vt;
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Similar to ZeroVtoSlidingRegion, a velocity vt within the stiction
// region (but not to zero) is updated to a sliding configuration. Since vt
// falls in a region of strong gradients, the limiter allows it.
TEST_F(DirectionLimiter, WithinStictionRegionToSlidingRegion) {
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7) * v_stiction;
  const Vector2<double> dvt = Vector2<double>(0.9, -0.3);
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Similar to test ZeroVtoSlidingRegion, but vt is not exactly zero
// but negligibly small with norm/v_stiction < tolerance.
TEST_F(DirectionLimiter, StictionToSliding) {
  const Vector2<double> vt =
      Vector2<double>(-0.5, 0.3) * v_stiction * tolerance;
  const Vector2<double> dvt(0.3, 0.15);

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  // For this case TalsLimiter neglects the very small initial vt
  // (since we always have tolerance << 1.0) so that:
  // vα = vt + αΔvt ≈ αΔvt = Δvt/‖Δvt‖⋅vₛ/2.
  // Therefore we expect α = 1 / ‖Δvt‖⋅vₛ/2.
  double alpha_expected = 1.0 / dvt.norm() * v_stiction / 2.0;

  EXPECT_NEAR(alpha, alpha_expected, kTolerance);
}

// Verifies that the limiter allows negligible changes dvt with alpha = 1.
TEST_F(DirectionLimiter, VerySmallDeltaV) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt =
      Vector2<double>(-0.5, 0.3) * v_stiction * tolerance;
  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// A very specific scenario when the update vt + dvt crosses zero exactly.
// This is a very common case in 1D-like problems and therefore it does happen
// often.
TEST_F(DirectionLimiter, StraightCrossThroughZero) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt(-0.3, -0.15);  // dvt = -3 * vt.

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  // Since the change crosses zero exactly, we expect
  // v_alpha = v + alpha * dv = v/‖v‖⋅vₛ/2.
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stiction / 2.0;

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Test a direction change from vt to v1 = vt + dvt that crosses through the
// stiction region. In this case the limiter will find a scalar 0 < alpha < 1
// such that v_alpha = vt + alpha * dvt is the closest vector to the origin.
TEST_F(DirectionLimiter, CrossStictionRegionFromTheOutside) {
  // We construct a v_alpha expected to be within the stiction region.
  const Vector2<double> vt_alpha_expected =
      Vector2<double>(0.3, 0.2) * v_stiction;

  // A unit vector normal to vt_alpha_expected.
  const Vector2<double> vt_normal =
      Vector2<double>(vt_alpha_expected(1), -vt_alpha_expected(0)).normalized();

  // Construct a vt away from the circle in a large magnitude (>>v_s) in
  // the direction normal to vt_alpha_expected.
  const Vector2<double> vt = vt_alpha_expected + 0.5 * vt_normal;

  // Construct a v1 away from the circle in a large magnitude (>>v_s) in
  // the direction normal to vt_alpha_expected. This time in the opposite
  // direction to that of vt.
  const Vector2<double> v1 = vt_alpha_expected - 0.8 * vt_normal;

  // Velocity change from vt to v1 (this is what the TAMSI iteration
  // would compute).
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  // Verify the result from the limiter.
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the stiction region. In this test, the angle formed by vt and v1 is
// smaller than theta_max and the limiter allows it, i.e. it returns alpha = 1.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion) {
  // an angle smaller that theta_max = M_PI / 6.
  const double theta = M_PI / 8.0;

  // A vt outside the stiction region
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the stiction region. In this test, the angle formed by vt and v1 is
// larger than theta_max and the limiter will limit the change to a
// vt_alpha = vt + alpha * dvt so that vt_alpha forms an angle theta_max with
// vt.
// Internal detail note: the limiter computed two roots with different signs and
// returns the positive root.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion_LargeTheta) {
  // Angle formed by v1 and vt, an angle larger that theta_max = M_PI / 6.
  const double theta1 = M_PI / 3.0;

  // A vt outside the stiction region
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta1) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the stiction region. In this test, the angle formed by vt and v1 is
// MUCH larger than theta_max and the limiter will limit the change to a
// vt_alpha = vt + alpha * dvt so that vt_alpha forms an angle theta_max with
// vt.
// Internal detail note: the limiter computed two positive roots and returns
// the smallest of the two.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion_VeryLargeTheta) {
  // Angle formed by v1 and vt, an angle larger that theta_max = M_PI / 6.
  const double theta1 = 5.0 * M_PI / 6.0;

  // A vt outside the stiction region
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta1) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

// This is a very degenerate case in which the angle formed by vt and dvt
// equals (within machine epsilon) to theta_max. It so happens that in this
// case there is a single solution to the quadratic equation solved by the
// limiter (the equation becomes linear).
// Even though this will rarely (or impossibly) happen, we make sure we consider
// it for maximum robustness.
TEST_F(DirectionLimiter, ChangesWithinTheSlidingRegion_SingleSolution) {
  // A vt outside the stiction region
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A dvt forming an angle of theta_max with vt not necessarily having the same
  // magnitude as vt.
  const Vector2<double> dvt = -3.0 * Rotation(theta_max) * vt;

  // Before proceeding with the test, assert that we are in a case where theta1
  // is larger than theta_max.
  const Vector2<double> v1 = vt + dvt;
  double theta1 = std::cos(vt.dot(v1)/vt.norm()/v1.norm());
  ASSERT_GT(theta1, theta_max);

  const double alpha = internal::TalsLimiter<double>::CalcAlpha(
      vt, dvt, cos_min, v_stiction, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

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
any point to the triangle's center is 2h/3. The height h relates to the length
a of a side by h = 3/2/sqrt(3) a.
The generalized positions vector for this case is q = [x, y, theta], with
theta = 0 for the triangle in the configuration shown in the schematic. */
class PizzaSaver : public ::testing::Test {
 public:
  void SetUp() override {
    // Now we'll set up each term in the equation:
    //   Mv̇ = τ + Dᵀ fₜ
    // where τ =[Fx, Fy, Mz] contains the external force in x, the external
    // force in y and the external moment about z (out of plane).
    M_ << m_,  0,  0,
           0, m_,  0,
           0,  0,  I_;
  }

  MatrixX<double> ComputeTangentialJacobian(double theta) {
    MatrixX<double> Jt(2 * nc_, nv_);
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
    Jt.block(0, 0, 2, nv_) << 1, 0, -p_BoA_W.y(),
        0, 1,  p_BoA_W.x();

    // Point B
    Jt.block(2, 0, 2, nv_) << 1, 0, -p_BoB_W.y(),
        0, 1,  p_BoB_W.x();

    // Point C
    Jt.block(4, 0, 2, nv_) << 1, 0, -p_BoC_W.y(),
        0, 1,  p_BoC_W.x();

    return Jt;
  }

  void SetProblem(const Vector3<double>& v0, const Vector3<double>& tau,
                  double mu, double theta, double dt) {
    // Next time step generalized momentum if there are no friction forces.
    p_star_ = M_ * v0 + dt * tau;

    // Normal forces. Assume they are equally distributed.
    fn_ = m_ * g_ / 3.0 * Vector3<double>::Ones();

    // All contact points have the same friction for this case.
    mu_ = mu * Vector3<double>::Ones();

    // The generalized velocites do not affect the out-of-plane separation
    // velocities for this problem. Normal forces are decoupled.
    Jn_.setZero();

    Jt_ = ComputeTangentialJacobian(theta);

    solver_.SetOneWayCoupledProblemData(&M_, &Jn_, &Jt_, &p_star_, &fn_, &mu_);
  }

  void SetNoContactProblem(const Vector3<double>& v0,
                           const Vector3<double>& tau,
                           double dt) {
    // Next time step generalized momentum if there are no friction forces.
    p_star_ = M_ * v0 + dt * tau;

    // No contact points.
    fn_.resize(0);
    mu_.resize(0);
    Jn_.resize(0, nv_);
    Jt_.resize(0, nv_);

    solver_.SetOneWayCoupledProblemData(&M_, &Jn_, &Jt_, &p_star_, &fn_, &mu_);
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

  // The separation velocities Jacobian.
  MatrixX<double> Jn_{nc_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> Jt_{2 * nc_, nv_};

  // The TAMSI solver for this problem.
  TamsiSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  Vector3<double> p_star_;  // Generalized momentum.
  VectorX<double> fn_;      // Normal forces at each contact point.
  VectorX<double> mu_;      // Friction coefficient at each contact point.
};

// This tests the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (that is, the sliding
// velocity should be smaller than the regularization parameter). Otherwise the
// saver will start sliding. For this setup the transition occurs at
// M_transition = mu * m * g * R = 5.0
TEST_F(PizzaSaver, SmallAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution (meaning the result is independent of angle theta).
  const double theta = M_PI / 5;

  // External forcing.
  const double Mz = 3.0;  // M_transition = 5.0
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetProblem(v0, tau, mu, theta, dt);

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  parameters.relative_tolerance = 1.0e-4;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      // Dimensionless relative (to the stiction tolerance) tolerance.
      solver_.get_solver_parameters().relative_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction should balance the applied Mz. However, it will
  // take several time steps until tau_f balances Mz (eventually it will).
  // Therefore, here we just sanity check that Mz is at least relatively close
  // (to the value of Mz) to tau_f. In other words, with only a single time
  // step, we are still accelerating towards the final steady state slip
  // introduced by having a finite stiction tolerance.
  EXPECT_NEAR(tau_f(2), -Mz, 5.0e-4);

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

  // For this case where Mz < M_transition, we expect stiction (slip velocities
  // are smaller than the regularization parameter).
  EXPECT_LT(v_slipA, parameters.stiction_tolerance);
  EXPECT_LT(v_slipB, parameters.stiction_tolerance);
  EXPECT_LT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
  // solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.relative_tolerance;
  MatrixX<double> J_expected = test::CalcOneWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, mu_, fn_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

// Exactly the same problem as in PizzaSaver::SmallAppliedMoment but with an
// applied moment Mz = 6.0 > M_transition = 5.0. In this case the pizza saver
// transitions to sliding with a net moment of Mz - M_transition during a
// period (time stepping interval) dt. Therefore we expect a change of angular
// velocity given by Δω = dt (Mz - Mtransition) / I.
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

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  parameters.relative_tolerance = 1.0e-4;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      // Dimensionless relative (to the stiction tolerance) tolerance.
      solver_.get_solver_parameters().relative_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);
  // Since we are sliding, the total moment should match M_transition.
  // The difference with Mz is what makes the saver to start accelerating.
  EXPECT_NEAR(tau_f(2), -M_transition, 1.0e-13);

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

  // For this problem we expect the translational velocities for the COM to be
  // zero. Still, there is slip at points A, B, C.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  const double omega = dt * (Mz - 5.0) / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // Slip velocities should only be due to rotation.
  EXPECT_NEAR(v_slipA, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipB, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipC, R_ * omega, kTolerance);

  // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
  // solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.relative_tolerance;
  MatrixX<double> J_expected = test::CalcOneWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, mu_, fn_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance = J_expected.rows() * J_expected.norm() *
      std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

// Verify the solver behaves correctly when the problem data contains no
// contact points.
TEST_F(PizzaSaver, NoContact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.

  // External forcing.
  const double Mz = 6.0;
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetNoContactProblem(v0, tau, dt);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  EXPECT_EQ(solver_.get_generalized_friction_forces(), Vector3<double>::Zero());

  const auto& stats = solver_.get_iteration_statistics();
  EXPECT_EQ(stats.vt_residual(), 0);
  EXPECT_EQ(stats.num_iterations, 1);

  // Verify solution.
  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);
  // Expected angular velocity change about z due to the applied moment Mz.
  const double omega = dt * Mz / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // No contact.
  EXPECT_EQ(solver_.get_tangential_velocities().size(), 0);
}

// This test verifies that TAMSI can correctly predict transitions in a problem
// with impact. In this test the y axis is in the "up" vertical direction, the x
// axis points to the right and the z axis comes out of the x-y plane forming a
// right handed basis. Gravity points in the minus y direction. The x-z plane is
// the ground.
// In this test a cylinder moves parallel to the ground with its revolute axis
// parallel to the z axis at all times. The cylinder's COM is constrained to
// move in the x-y plane. Therefore, the cylinder's motions are described by
// three degrees of freedom (DOFs); two translational DOFs in the x-y plane and
// a rotational DOF about its revolute axis. The cylinder has radius R,
// mass m, and rotational inertia I, and it is initially at height y0 = h0 + R
// from a flat ground.
// At t = 0, the cylinder is given an initial horizontal velocity vx0 and zero
// vertical velocity vy0 = 0. The cylinder will undergo a parabolic free flight
// towards the ground until the moment of impact at which the vertical velocity
// goes to zero (a purely inelastic collision). Since we know the initial
// height, we therefore know that the vertical velocity at the time of impact
// will be vy = -sqrt(2 g h0), with g the acceleration of gravity. Therefore the
// change of momentum, in the vertical direction, at the time of impact will be
// py = m * vy.
// TAMSI needs to know the normal forces in advance. We compute the normal force
// in order to exactly bring the cylinder to a stop in the vertical direction
// within a time interval dt from the time that the bodies first make contact.
// That is, we set the normal force to fn = -m * vy / dt + m * g, where the
// small contribution due to gravity is needed to exactly bring the cylinder's
// vertical velocity to zero. The solver keeps this value constant throughout
// the computation.
// The equations governing the motion for the cylinder during impact are:
//   (1)  I Δω = pt R,  Δω  = ω, since ω0 = 0.
//   (2)  m Δvx = pt ,  Δvx = vx - vx0
//   (3)  vt = vx + ω R
//   (4)  |pt| ≤ μ pn
// where pt = dt ft and pn = dt fn are the impulses due to friction (in the
// tangential direction) and due to the normal force, respectively.
// The problem above can be solved analytically for vx, ω and ft. We will find
// the condition for either stiction or sliding after impact by solving the
// easier stiction problem and subsequently verifying the condition given by
// Eq. (4).
//
//                  Stiction After Impact, vt = 0.
// Setting vt = 0 in Eq. (3), solving for ω and substituting the result in
// Eq. (1) leads now, together with Eq. (2), to a system of equations in vx and
// pt. We solve it for pt to find:
//   pt = -m vx0 / (1 + m R²/I)
// From Eq. (4), stiction occurs if vx0 < vx_transition, with vx_transition
// defined as:
//   vx_transition =  μ (1 + m R²/I) pn/m
// Otherwise the cylinder will be sliding after impact.
class RollingCylinder : public ::testing::Test {
 public:
  void SetUp() override {
    // Mass matrix corresponding to free (in 2D) cylinder.
    // Generalized velocities are v = [vx, vy, ω].
    M_ << m_,  0,  0,
           0, m_,  0,
           0,  0,  I_;
  }

  // Computes tangential velocity Jacobian s.t. vt = Jt * v.
  // Where vt is a vector in ℝ². Its first component corresponds to the
  // tangential velocity along the x-axis and its second component corresponds
  // to the out-of-plane (z-axis) tangential velocity. Since the problem is 2D,
  // the second component along the z axis is zero always.
  MatrixX<double> ComputeTangentialJacobian() {
    MatrixX<double> D(2, nv_);
    // vt = vx + w * R = [1, 0, R] * v
    D << 1.0, 0.0, R_,   // Along the x axis
        0.0, 0.0, 0.0;  // Along the z axis out of the plane.
    return D;
  }

  // Sets the TamsiSolver to solve this cylinder case from the
  // input data:
  //   v0: velocity right before impact.
  //   tau: vector of externally applied generalized forces.
  //   mu: friction coefficient between the cylinder and the ground.
  //   height: the initial height the cylinder is dropped from.
  //   dt: time step used by the solver.
  //   num_contacts_multiplier: for testing purposes, we repeat the same contact
  //   point num_contacts_multiplier times. This allow us to test how the solver
  //   does in situations with multiple points of contact even if coincident.
  void SetImpactProblem(const Vector3<double>& v0, const Vector3<double>& tau,
                        double mu, double height, double dt,
                        int num_contacts_multiplier = 1) {
    // Next time step generalized momentum if there are no contact forces.
    p_star_ = M_ * v0 + dt * tau;

    // This problem has a single contact point. We multiply it to emulate
    // a system with multiple points of contact.
    nc_ = num_contacts_multiplier;

    // Friction coefficient for the only contact point in the problem.
    mu_vector_ = VectorX<double>::Constant(nc_, mu);

    // Compute Jacobian matrices.
    Jn_.resize(nc_, nv_);
    Jn_ = RowVector3<double>(0, 1, 0).replicate(num_contacts_multiplier, 1);
    Jt_.resize(2 * nc_, nv_);
    Jt_ = ComputeTangentialJacobian().replicate(num_contacts_multiplier, 1);

    // A very small penetration allowance for practical purposes.
    const double penetration_allowance = 1.0e-6;
    // Initial penetration of O(dt), in tests below we use dt = 1.0e-3.
    const double xini = 1.0e-3;
    // We evenly distribute xinitial among all contact points.
    x0_ = VectorX<double>::Constant(nc_, xini / nc_);
    const double k = m_ * g_ / penetration_allowance;
    stiffness_ = VectorX<double>::Constant(nc_, k);
    fn0_ = stiffness_.array() * x0_.array();
    const double omega = sqrt(stiffness_(0) / m_);
    const double time_scale = 1.0 / omega;
    const double damping_ratio = 1.0;
    const double dissipation =
        damping_ratio * time_scale / penetration_allowance;
    dissipation_ = VectorX<double>::Constant(nc_, dissipation);

    solver_.SetTwoWayCoupledProblemData(&M_, &Jn_, &Jt_, &p_star_, &fn0_,
                                        &stiffness_, &dissipation_,
                                        &mu_vector_);
  }

 protected:
  // For this unit test we have:
  //   R = 1.0
  //   m = 1.0
  //   I = 1.0
  //   h0 = 1 / 2
  //   g = 9.0
  //   mu = 0.1
  // With this set of parameters we have:
  //   vy = -3.0 m/s (velocity at impact).
  //   pn = 3.0 Ns
  //   vx_transition = 0.6 m/s
  // with pn = −m vy = m sqrt(2 g h0) and vx_transition determined from
  // vx_transition =  μ (1 + m R²/I) pn/m as described in the documentation of
  // this test fixture.
  const double m_{1.0};   // Mass of the cylinder, kg.
  const double R_{1.0};   // Radius of the cylinder, m.
  const double g_{9.0};   // Acceleration of gravity, m/s².
  // For a thin cylindrical shell the moment of inertia is I = m R². We use this
  // inertia so that numbers are simpler for debugging purposes
  // (I = 1.0 kg m² in this case).
  const double I_{R_ * R_ * m_};  // kg m².
  const double mu_{0.1};  // Coefficient of friction, dimensionless.

  // Problem sizes.
  const int nv_{3};  // number of generalized velocities.
  int nc_{1};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> Jt_;

  // Normal separation velocities Jacobian.
  MatrixX<double> Jn_;

  VectorX<double> stiffness_;
  VectorX<double> dissipation_;
  VectorX<double> x0_;
  VectorX<double> fn0_;

  // TAMSI solver for this problem.
  TamsiSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  VectorX<double> p_star_{nv_};  // Generalized momentum.
  VectorX<double> mu_vector_;  // Friction at each contact point.
};

TEST_F(RollingCylinder, StictionAfterImpact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.1;  // Friction coefficient.

  // Other than normal contact forces, external forcing for this problem
  // includes gravity.
  const Vector3<double> tau(0.0, -m_ * g_, 0.0);

  // Initial height. We choose it so that vy at impact is exactly 3.0 m/s.
  const double h0 = 0.5;

  // Vertical velocity at the moment of impact.
  const double vy0 = -sqrt(2.0 * g_ * h0);

  // Initial horizontal velocity (recall vx_transition = 0.6 m/s)
  const double vx0 = 0.5;  // m/s.

  // Initial velocity.
  const Vector3<double> v0(vx0, vy0, 0.0);

  // We solve exactly the same problem but repeating the contact point multiple
  // times to emulate multi-point contact.
  for (int num_contacts = 1; num_contacts <= 256; num_contacts *= 4) {
    SetImpactProblem(v0, tau, mu, h0, dt, num_contacts);

    // Verify solver has allocated the proper workspace size.
    EXPECT_GE(TamsiSolverTester::get_capacity(solver_), num_contacts);

    TamsiSolverParameters parameters;  // Default parameters.
    parameters.stiction_tolerance = 1.0e-6;
    solver_.set_solver_parameters(parameters);

    TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
    ASSERT_EQ(info, TamsiSolverResult::kSuccess);

    VectorX<double> tau_f = solver_.get_generalized_friction_forces();

    const auto& stats = solver_.get_iteration_statistics();

    const double vt_tolerance =
        solver_.get_solver_parameters().relative_tolerance *
        solver_.get_solver_parameters().stiction_tolerance;
    EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

    // Friction should only act horizontally.
    EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

    // The moment due to friction Mf should exactly match R * ft.
    EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

    const VectorX<double>& vt = solver_.get_tangential_velocities();
    ASSERT_EQ(vt.size(), 2 * num_contacts);
    for (int ic = 0; ic < num_contacts; ++ic) {
      // There should be no spurious out-of-plane tangential velocity.
      EXPECT_NEAR(vt(2 * ic + 1), 0.0, kTolerance);

      // We expect stiction, to within the stiction tolerance.
      EXPECT_LT(std::abs(vt(2 * ic)), parameters.stiction_tolerance);
    }

    const VectorX<double>& v = solver_.get_generalized_velocities();
    ASSERT_EQ(v.size(), nv_);

    // We expect rolling, i.e. vt = vx + omega * R = 0, to within the stiction
    // tolerance.
    EXPECT_LT(std::abs(v(0) + R_ * v(2)), parameters.stiction_tolerance);

    // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
    // solver's internal implementation.
    MatrixX<double> J = TamsiSolverTester::CalcJacobian(solver_, v, dt);

    // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
    // a completely separate implementation using automatic differentiation.
    const double v_stiction = parameters.stiction_tolerance;
    const double epsilon_v = v_stiction * parameters.relative_tolerance;
    MatrixX<double> J_expected = test::CalcTwoWayCoupledJacobianWithAutoDiff(
        M_, Jn_, Jt_, p_star_, x0_, mu_vector_, stiffness_, dissipation_, dt,
        v_stiction, epsilon_v, v);

    // Verify the result.
    const double J_tolerance = 30 * std::numeric_limits<double>::epsilon();
    EXPECT_TRUE(CompareMatrices(J, J_expected, J_tolerance,
                                MatrixCompareType::relative));
  }
}

// Same tests a RollingCylinder::StictionAfterImpact but with a smaller friction
// coefficient of mu = 0.1 and initial horizontal velocity of vx0 = 1.0 m/s,
// which leads to the cylinder to be sliding after impact.
// This is a case for which the initial horizontal velocity is
// vx0 > vx_transition and therefore we expect sliding after impact.
TEST_F(RollingCylinder, SlidingAfterImpact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.1;     // Friction coefficient.

  // Other than normal contact forces, external forcing for this problem
  // includes gravity.
  const Vector3<double> tau(0.0, -m_ * g_, 0.0);

  // Initial height. We choose it so that vy at impact is exactly 3.0 m/s.
  const double h0 = 0.5;

  // Vertical velocity at the moment of impact.
  const double vy0 = -sqrt(2.0 * g_ * h0);

  // Initial horizontal velocity (vx_transition = 0.6 m/s).
  const double vx0 = 0.7;  // m/s.

  // Initial velocity.
  const Vector3<double> v0(vx0, vy0, 0.0);

  SetImpactProblem(v0, tau, mu, h0, dt);

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      solver_.get_solver_parameters().relative_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // Friction should only act horizontally.
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction Mf should exactly match R * ft.
  EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // There should be no spurious out-of-plane tangential velocity.
  EXPECT_NEAR(vt(1), 0.0, kTolerance);

  // We expect sliding, i.e. a (positive) sliding velocity larger than the
  // stiction tolerance.
  EXPECT_GT(vt(0), parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // Even though not rolling, the cylinder should be rotating clockwise.
  EXPECT_TRUE(v(2) < 0.0);

  // We expect the solver to update vt accordingly based on v before return.
  EXPECT_NEAR(v(0) + R_ * v(2), vt(0), kTolerance);

  // Compute the Newton-Raphson Jacobian of the (two-way coupled)
  // residual J = ∇ᵥR using the solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.relative_tolerance;
  MatrixX<double> J_expected = test::CalcTwoWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, x0_, mu_vector_,
      stiffness_, dissipation_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

GTEST_TEST(EmptyWorld, Solve) {
  const int nv = 0;
  TamsiSolver<double> solver{nv};

  // (Empty) problem data.
  VectorX<double> p_star, mu_vector, fn0, stiffness, dissipation;
  MatrixX<double> M, Jn, Jt;

  DRAKE_EXPECT_NO_THROW(solver.SetTwoWayCoupledProblemData(
      &M, &Jn, &Jt, &p_star, &fn0, &stiffness, &dissipation, &mu_vector));
}

}  // namespace
}  // namespace multibody
}  // namespace drake


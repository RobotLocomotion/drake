#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/implicit_stribeck/test/implicit_stribeck_solver_test_util.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace implicit_stribeck {
class ImplicitStribeckSolverTester {
 public:
  static MatrixX<double> CalcJacobian(
      ImplicitStribeckSolver<double>& solver,
      const Eigen::Ref<const VectorX<double>>& v,
      double dt) {
    // Problem data.
    const auto& M = *solver.problem_data_aliases_.M_ptr;
    const auto& Jn = *solver.problem_data_aliases_.Jn_ptr;
    const auto& Jt = *solver.problem_data_aliases_.Jt_ptr;

    // Workspace with size depending on the number of contact points.
    auto vn = solver.variable_size_workspace_.mutable_vn();
    auto vt = solver.variable_size_workspace_.mutable_vt();
    auto fn = solver.variable_size_workspace_.mutable_fn();
    auto Gn = solver.variable_size_workspace_.mutable_Gn();
    auto mus = solver.variable_size_workspace_.mutable_mu();
    auto t_hat = solver.variable_size_workspace_.mutable_t_hat();
    auto v_slip = solver.variable_size_workspace_.mutable_v_slip();
    auto& dft_dvt = solver.variable_size_workspace_.mutable_dft_dv();
    auto phi = solver.variable_size_workspace_.mutable_phi();

    // Normal separation velocity.
    vn = Jn * v;

    if (solver.two_way_coupling()) {
      const auto& phi0 = *solver.problem_data_aliases_.phi0_ptr;
      // Penetration distance (positive when there is penetration).
      phi = phi0 - dt * vn;
    }

    // Computes friction forces fn and gradients Gn as a funciton of phi, vn,
    // Jn and dt.
    solver.CalcNormalForces(phi, vn, Jn, dt, &fn, &Gn);

    // Tangential velocity.
    vt = Jt * v;

    // Update ft, mus, t_hat, v_slip as a function of vt, vn and, the problem
    // data.
    solver.CalcFrictionForces(vt, fn);

    // Compute gradient ∇ᵥₜfₜ(vₜ), dft_dvt in source, as a function of fn, mus,
    // t_hat, v_slip and, the problem data.
    solver.CalcFrictionForcesGradient(fn, mus, t_hat, v_slip);

    solver.CalcJacobian(M, Jn, Jt, Gn, dft_dvt, t_hat, mus, dt);

    return solver.fixed_size_workspace_.J;
  }
};
namespace {
#if 0
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

    solver_.SetProblemData(&M_, &Jn_, &Jt_, &p_star_, &fn_, &mu_);
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

    solver_.SetProblemData(&M_, &Jn_, &Jt_, &p_star_, &fn_, &mu_);
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

  // The implicit Stribeck solver for this problem.
  ImplicitStribeckSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  Vector3<double> p_star_;  // Generalized momentum.
  VectorX<double> fn_;      // Normal forces at each contact point.
  VectorX<double> mu_;      // Friction coefficient at each contact point.
};

// This tests the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (within the Stribeck
// approximation). Otherwise the saver will be sliding.
// For this setup the transition occurs at M_transition = mu * m * g * R = 5.0
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
  // introduce by having a finite stiction tolerance.
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

  // For this case where Mz < M_transition, we expect stiction (within the
  // Stribeck's stiction tolerance)
  EXPECT_LT(v_slipA, parameters.stiction_tolerance);
  EXPECT_LT(v_slipB, parameters.stiction_tolerance);
  EXPECT_LT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  // Verify the internal implementation of the solver to compute the Jacobian of
  // the Newton-Raphson residual, against a Jacobian computed using an
  // AutoDiffXd implementation.
  MatrixX<double> J =
      ImplicitStribeckSolverTester::CalcJacobian(solver_, v, dt);
  PRINT_VARn(J);

  // Non-used data by the one-way coupling scheme.
  const VectorX<double> not_used;
  const double v_stribeck = parameters.stiction_tolerance;
  const double epsilon_v = v_stribeck * parameters.tolerance;
  MatrixX<double> J_expected = CalcJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, not_used /*phi0*/, mu_, fn_,
      not_used /*stiffness*/, not_used /*damping*/,
      dt, v_stribeck, epsilon_v, false /* two-way coupling? */, v);

  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  PRINT_VAR(J_tolerance);

  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));

  PRINT_VARn(J_expected);

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
  parameters.tolerance = 1.0e-4;
  solver_.set_solver_parameters(parameters);

  ComputationInfo info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, ComputationInfo::Success);

  VectorX<double> tau_f = solver_.get_generalized_forces();

  const IterationStats& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      /* Dimensionless relative (to the stiction tolerance) tolerance */
      solver_.get_solver_parameters().tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);
  // Since we are sliding, the total moment should match M_transition.
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

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  const double omega = dt * (Mz - 5.0) / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // Slip velocities should only be due to rotation.
  EXPECT_NEAR(v_slipA, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipB, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipC, R_ * omega, kTolerance);
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

  ComputationInfo info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, ComputationInfo::Success);

  EXPECT_EQ(solver_.get_generalized_forces(), Vector3<double>::Zero());

  const IterationStats& stats = solver_.get_iteration_statistics();
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
#endif

// This test verifies that the implicit stribeck solver can correctly predict
// transitions in a problem with impact. This is a 2D problem consisting of a
// cylinder of radius R, mass m, and rotational inertia I, initially at height
// z0 = h0 + R from a flat ground.
// At t = 0, the cylinder is given an initial horizontal velocity vx0 and zero
// vertical velocity vy0 = 0. The cylinder will undergo a parabolic free flight
// towards the ground until the moment of impact at which the vertical velocity
// goes to zero (a purely inelastic collision). Since we know the initial
// height, we therefore know that the vertical velocity at the time of impact
// will be vy = sqrt(m⋅g⋅h₀), with g the acceleration of gravity. Therefore the
// change of momentum, in the vertical direction, at the time of impact will be
// py = -m * vy.
// The implicit Stribeck solver needs to know the normal forces in advance. We
// compute the normal force in order to exactly bring the cylinder to a stop in
// the vertical direction within a time interval dt. That is, we set the normal
// force to fn = m * vy / dt + m * g, where the small contribution due to
// gravity is needed to exactly bring the cylinder's vertical velocity to zero.
// The equations governing the motion of the cylinder within dt of the time of
// impact are:
//   (1)  I⋅Δω = pt⋅R,  Δω  = ω, since ω0 = 0.
//   (2)  m⋅Δvx = pt, Δvx = vx - vx0
//   (3)  vt = vx + ω⋅R
//   (4)  |pt| ≤ μ⋅pn
// where pt = dt⋅ft and pn = dt⋅fn are the impulses due to friction and the
// normal force, respectively.
// The problem above can be solved analytically for vx, ω and ft. We will find
// the condition for either stiction or sliding after impact by solving the
// easier stiction problem and subsequently verifying the condition given by
// Eq. (4).
//
//                  Stiction After Impact, vt = 0.
// Setting vt = 0 in Eq. (3), solving for ω and substituting the result in
// Eq. (1) leads now, together with Eq. (2), to a system of equations in vx and
// pt. We solve it for pt to find:
//   pt = -m⋅vx0 / (1 + m⋅R²/I)
// From Eq. (4), stiction occurs if vx0 is smaller than:
//   vx0 ≤ vx_transition =  μ⋅(1 + m⋅R²/I)⋅pn/m
// Otherwise the cylinder will be sliding after impact.
//
//              Transition of this test's problem parameters.
//
// For this unit test we have:
//   R = 1.0
//   m = 1.0
//   I = 1.0
//   h0 = 1 / 2
//   g = 9.0
//   mu = 0.1
// With this set of parameters we have:
//   vy = 3.0 m/s (velocity at impact).
//   pn = 3.0 Ns
//   vx_transition = 0.6 m/s
class RollingCylinder : public ::testing::Test {
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

  MatrixX<double> ComputeTangentialJacobian() {
    MatrixX<double> D(2 * nc_, nv_);
    // vt = vx + w * R = [1, 0, R] * v
    D << 1.0, 0.0, R_,   // Along the x axis
        0.0, 0.0, 0.0;  // Along the z axis out of the plane.
    return D;
  }

  void SetImpactProblem(const Vector3<double>& v0, const Vector3<double>& tau,
                        double mu, double height, double dt) {
    // Next time step generalized momentum if there are no contact forces.
    p_star_ = M_ * v0 + dt * tau;// + dt * Vector3<double>(0.0, fn_(0), 0.0);

    // Friction coefficient for the only contact point in the problem.
    mu_(0) = mu;

    // Compute Jacobian matrices.
    N_ << 0, 1, 0;
    D_ = ComputeTangentialJacobian();

    // A very small penetration allowance for practical purposes.
    const double penetration_allowance = 1.0e-6;
    // Initial penetration of O(dt), in tests below we use dt = 1.0e-3.
    phi0_(0) = 1.0e-3;
    stiffness_(0) = m_ * g_ / penetration_allowance;
    const double omega = sqrt(stiffness_(0) / m_);
    const double time_scale = 1.0 / omega;
    const double damping_ratio = 1.0;
    const double damping = damping_ratio * time_scale / penetration_allowance;
    damping_(0) = damping;

    PRINT_VAR(stiffness_);
    PRINT_VAR(damping_);
    PRINT_VAR(phi0_);

    solver_.SetTwoWayCoupledProblemData(
        &M_, &N_, &D_, &p_star_, &phi0_, &stiffness_, &damping_, &mu_);
  }

 protected:
  // Problem parameters.
  const double m_{1.0};   // Mass of the cylinder.
  const double R_{1.0};   // Distance from COM to any contact point.
  const double g_{9.0};  // Acceleration of gravity.
  // For a thin cylindrical shell the moment of inertia is I = m⋅R². We use this
  // inertia so that numbers are simpler (I = 1.0 kg⋅m² in this case).
  const double I_{R_ * R_ * m_};

  // Problem sizes.
  const int nv_{3};  // number of generalized velocities.
  const int nc_{1};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> D_{2 * nc_, nv_};

  // Normal separation velocities Jacobian.
  MatrixX<double> N_{nc_, nv_};

  VectorX<double> stiffness_{nc_};
  VectorX<double> damping_{nc_};
  VectorX<double> phi0_{nc_};

  // The implicit Stribeck solver for this problem.
  ImplicitStribeckSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  VectorX<double> p_star_{nv_};  // Generalized momentum.
  VectorX<double> mu_{nc_};      // Friction coefficient at each contact point.
};

TEST_F(RollingCylinder, StictionAfterImpact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  (void) kTolerance;

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

  SetImpactProblem(v0, tau, mu, h0, dt);

  Parameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  ComputationInfo info = solver_.SolveWithGuess(dt, v0);
  EXPECT_EQ(info, ComputationInfo::Success);

  VectorX<double> tau_f = solver_.get_generalized_forces();

  const IterationStats& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      /* Dimensionless relative (to the stiction tolerance) tolerance */
      solver_.get_solver_parameters().tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  PRINT_VAR(stats.num_iterations);
  PRINT_VAR(stats.vt_residual());

  // Friction should only act horizontally.
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction Mf should exactly match R * ft.
  EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // There should be no spurious out-of-plane tangential velocity.
  EXPECT_NEAR(vt(1), 0.0, kTolerance);

  PRINT_VAR(vt(1));

  // We expect stiction, to within the Stribeck stiction tolerance.
  EXPECT_LT(vt(0), parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // Since we imposed the normal force exactly to bring the cylinder to a stop,
  // we expect the vertical velocity to be zero.
  //EXPECT_NEAR(v(1), 0.0, kTolerance);

  // We expect rolling, i.e. vt = vx + omega * R = 0, to within the stiction
  // tolerance.
  EXPECT_LT(std::abs(v(0) + R_ * v(2)), parameters.stiction_tolerance);



  MatrixX<double> J =
      ImplicitStribeckSolverTester::CalcJacobian(solver_, v, dt);
  PRINT_VARn(J);


  const VectorX<double> na;  // Non-used data for one-way coupling.
  const double v_stribeck = parameters.stiction_tolerance;
  const double epsilon_v = v_stribeck * parameters.tolerance;
  const VectorX<double> not_used;  // variables not used in two-way coupling.
  MatrixX<double> J_expected = CalcJacobianWithAutoDiff(
      M_, N_, D_, p_star_, phi0_, mu_, not_used /* fn */,
      stiffness_, damping_, dt, v_stribeck, epsilon_v, true, v);

  PRINT_VARn(J_expected);

  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  PRINT_VAR(J_tolerance);

  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

// Same tests a RollingCylinder::StictionAfterImpact but with a smaller friction
// coefficient of mu = 0.1 and initial horizontal velocity of vx0 = 1.0 m/s,
// which leads to the cylinder to be sliding after impact.
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
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // Friction should only act horizontally.
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction Mf should exactly match R * ft.
  EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // There should be no spurious out-of-plane tangential velocity.
  EXPECT_NEAR(vt(1), 0.0, kTolerance);

  // We expect sliding, i.e. a sliding velocity larger than the Stribeck
  // stiction tolerance.
  EXPECT_GT(vt(0), parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // Even though not rolling, the cylinder should be rotating clockwise.
  EXPECT_TRUE(v(2) < 0.0);

  // We expect the solver to update vt accordingly based on v before return.
  EXPECT_NEAR(v(0) + R_ * v(2), vt(0), kTolerance);

  MatrixX<double> J =
      ImplicitStribeckSolverTester::CalcJacobian(solver_, v, dt);
  PRINT_VARn(J);

  const VectorX<double> na;  // Non-used data for one-way coupling.
  const double v_stribeck = parameters.stiction_tolerance;
  const double epsilon_v = v_stribeck * parameters.tolerance;
  const VectorX<double> not_used;  // variables not used in two-way coupling.
  MatrixX<double> J_expected = CalcJacobianWithAutoDiff(
      M_, N_, D_, p_star_, phi0_, mu_, not_used /* fn */,
      stiffness_, damping_, dt, v_stribeck, epsilon_v, true, v);

  PRINT_VARn(J_expected);

  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  PRINT_VAR(J_tolerance);

  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake


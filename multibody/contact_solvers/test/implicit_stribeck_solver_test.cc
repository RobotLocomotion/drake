#include "drake/multibody/contact_solvers/implicit_stribeck_solver.h"

#include <memory>

#include <gtest/gtest.h>

//#include "drake/common/test_utilities/expect_throws_message.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace {

// Top view of the pizza saver:
//
//   ^ y               C
//   |                 ◯
//   |                /\
//   ----> x         /  \
//                b /    \ a
//                 /      \
//                /        \
//               ◯----------◯
//               A     c    B
//
// It is modeled as an equilateral triangle with a contact point at each of the
// legs. The total mass of the pizza saver is m and its rotational inertia about
// the triangle's barycenter is I.
// If h is the height of the triangle from any of its sides, the distance from
// any point to the triangle's center is 2h/3. The height h relates to the size
// a of a side by h = 3/2/sqrt(3) a.
// The generalized positions vector for this case is q = [x, y, theta], with
// theta = 0 for the triangle in the configuration shown in the schematic.
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
    const Vector2<double> p_BoB( sqrt(3) / 2.0, -0.5);
    const Vector2<double> p_BoC( 0.0, 1.0);

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

  ImplicitStribeckSolver<double>::Parameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  VectorX<double> tau_f = solver_.SolveWithGuess(dt, v0);

  const ImplicitStribeckSolver<double>::IterationStats& stats =
      solver_.get_iteration_statistics();

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
  PRINT_VAR(vt.transpose());

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

  ImplicitStribeckSolver<double>::Parameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  VectorX<double> tau_f = solver_.SolveWithGuess(dt, v0);

  const ImplicitStribeckSolver<double>::IterationStats& stats =
      solver_.get_iteration_statistics();

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
    // At the moment of impact the cylinder will have a vertical velocity
    // vy = sqrt(2 * h * g).
    const double vy = sqrt(2.0 * g_ * height);

    // The non-penetration constraint implies there will be a discrete change of
    // momentum equal to m * vy. We therefore compute a normal force so that the
    // cylinder undergoes this finite change of momentum and comes to a stop
    // within the discrete interval dt.
    fn_(0) = m_ * vy / dt + m_ * g_;

    // Next time step generalized momentum if there are no friction forces.
    p_star_ = M_ * v0 + dt * tau + dt * Vector3<double>(0.0, fn_(0), 0.0);

    // Friction coefficient for the only contact point in the problem.
    mu_(0) = mu;

    D_ = ComputeTangentialJacobian();

    solver_.SetProblemData(&M_, &D_, &p_star_, &fn_, &mu_);
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

  // The implicit Stribeck solver for this problem.
  ImplicitStribeckSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  VectorX<double> p_star_{nv_};  // Generalized momentum.
  VectorX<double> fn_{nc_};      // Normal forces at each contact point.
  VectorX<double> mu_{nc_};      // Friction coefficient at each contact point.
};

TEST_F(RollingCylinder, StictionAfterImpact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  (void) kTolerance;

  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;  // Friction coefficient.

  // Other than normal contact forces, external forcing for this problem
  // includes gravity.
  const Vector3<double> tau(0.0, -m_ * g_, 0.0);

  // Initial height. We choose it so that vy at impact is exactly 3.0 m/s.
  const double h0 = 0.5;

  // Vertical velocity at the moment of impact.
  const double vy0 = -sqrt(2.0 * g_ * h0);

  // Initial horizontal velocity.
  const double vx0 = 0.1;  // m/s.

  // Initial velocity.
  const Vector3<double> v0(vx0, vy0, 0.0);

  SetImpactProblem(v0, tau, mu, h0, dt);

  ImplicitStribeckSolver<double>::Parameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  VectorX<double> tau_f = solver_.SolveWithGuess(dt, v0);

  const ImplicitStribeckSolver<double>::IterationStats& stats =
      solver_.get_iteration_statistics();

  const double vt_tolerance =
      /* Dimensionless relative (to the stiction tolerance) tolerance */
      solver_.get_solver_parameters().tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual < vt_tolerance);

  // Friction should only act horizontally.
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction Mf should exactly match R * ft.
  EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // There should be no spurious out-of-plane tangential velocity.
  EXPECT_NEAR(vt(1), 0.0, kTolerance);

  // We expect stiction, to within the Stribeck stiction tolerance.
  EXPECT_LT(vt(0), parameters.stiction_tolerance);

  PRINT_VAR(vt.transpose());

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // Since we imposed the normal force exactly to bring the cylinder to a stop,
  // we expect the vertical velocity to be zero.
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  // We expect rolling, i.e. vt = vx + omega * R = 0, to within the stiction
  // tolerance.
  EXPECT_LT(abs(v(0) + R_ * v(2)), parameters.stiction_tolerance);
}

}  // namespace
}  // namespace multibody
}  // namespace drake


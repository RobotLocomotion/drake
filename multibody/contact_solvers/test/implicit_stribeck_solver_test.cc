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
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    // The radius of the circumscribed circle R is the distance from each
    // contact point to the triangle's center.
    // If we model the pizza saver as three point masses m/3 at each contact
    // point, the moment of inertia is I = 3 * (m/3 R²):
    const double I = R_ * R_ * m_;  // = 1.0 in this case.

    // Now we'll set up each term in the equation:
    //   Mv̇ = τ + Dᵀ⋅fₜ
    // where τ =[Fx, Fy, Mz] contains the external force in x, the external
    // force in y and the external moment about z (out of plane).

    M_ << m_,  0,  0,
           0, m_,  0,
           0,  0,  I;
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


 protected:
  // Problem parameters.
  const double m_{1.0};   // Mass of the pizza saver.
  const double R_{1.0};   // Distance from COM to any contact point.
  const double g_{10.0};  // Acceleration of gravity.

  // Problem sizes.
  const int nv_{3};  // number of generalized velocities.
  const int nc_{3};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};
};

// This test the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (within the Stribeck
// approximation). Otherwise the saver will be sliding.
// For this setup the transition occurs at M_transition = mu * m * g * R = 5.0
TEST_F(PizzaSaver, SmallAppliedMoment) {
  PRINT_VARn(M_);
  PRINT_VARn(ComputeTangentialJacobian(0.0));

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

  // Next time step generalized momentum if there are no friction forces.
  const Vector3<double> p_star = M_ * v0 + dt * tau;

  // Normal forces. Assume they are equally distributed.
  const Vector3<double> fn = m_ * g_ / 3.0 * Vector3<double>::Ones();

  // All contact points have the same friction.
  const Vector3<double> mus = mu * Vector3<double>::Ones();

  const MatrixX<double> D = ComputeTangentialJacobian(theta);

  ImplicitStribeckSolver<double> solver(nv_);
  ImplicitStribeckSolver<double>::Parameters parameters;
  parameters.stiction_tolerance = 1.0e-4;
  solver.set_solver_parameters(parameters);
  solver.SetProblemData(&M_, &D, &p_star, &fn, &mus);

  VectorX<double> tau_f = solver.SolveWithGuess(dt, v0);

  const ImplicitStribeckSolver<double>::IterationStats& stats =
      solver.get_iteration_statistics();

  EXPECT_TRUE(stats.vt_residual < solver.get_solver_parameters().tolerance);

  PRINT_VAR(tau_f.transpose());
  PRINT_VAR(stats.vt_residual);
  PRINT_VAR(stats.num_iterations);
  PRINT_VAR(stats.linear_residuals.size());
  PRINT_VAR(stats.linear_iterations.size());
  for(int iter=0;iter<stats.num_iterations;++iter) {
    PRINT_VAR(iter);
    PRINT_VAR(stats.residuals[iter]);
    PRINT_VAR(stats.linear_iterations[iter]);
    PRINT_VAR(stats.linear_residuals[iter]);
    PRINT_VAR(stats.alphas[iter]);
  }

  const VectorX<double>& vt = solver.get_tangential_velocities();
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

  PRINT_VAR(v_slipA);
  PRINT_VAR(v_slipB);
  PRINT_VAR(v_slipC);

  const VectorX<double>& v = solver.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);
  PRINT_VAR(v.transpose());

  // For this problem we expect the translational velocities to be practically
  // zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);
}


}  // namespace
}  // namespace multibody
}  // namespace drake


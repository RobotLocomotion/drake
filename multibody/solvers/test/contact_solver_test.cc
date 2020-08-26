#include "drake/multibody/solvers/contact_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/multibody/solvers/test/contact_solver_driver.h"
#include "drake/multibody/solvers/contact_solver_utils.h"
#include "drake/multibody/solvers/test/pgs_solver.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {

using test::ContactSolverDriver;

namespace solvers {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

GTEST_TEST(ContactSolver, PizzaSaver) {
  const std::string model_file = "drake/multibody/solvers/test/pizza_saver.sdf";
  const double dt = 1.0e-3;
  ContactSolverDriver driver;
  driver.BuildModel(dt, model_file);  
  const auto& plant = driver.plant();
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();  
  ASSERT_EQ(nq, 7);
  ASSERT_EQ(nv, 6);

  const auto& body = plant.GetBodyByName("body");
  //driver.SetPointContactParameters(sphere, kStiffness, kDamping);
  auto& context = driver.mutable_plant_context();

  // For this test we want the ground to have the same friction as "body".
  const auto per_geometry_mu = driver.GetDynamicFrictionCoefficients(body);
  ASSERT_EQ(per_geometry_mu.size(), 3u);
  // Verify they are all equal.
  EXPECT_EQ(per_geometry_mu[0], per_geometry_mu[1]);
  EXPECT_EQ(per_geometry_mu[1], per_geometry_mu[2]);
  PRINT_VAR(per_geometry_mu[2]);
  driver.SetDynamicFrictionCoefficient(plant.world_body(), per_geometry_mu[0]);

  std::vector<ExternallyAppliedSpatialForce<double>> forces(1);
  forces[0].body_index = body.index();
  forces[0].p_BoBq_B = Vector3d::Zero();
  forces[0].F_Bq_W =
      SpatialForce<double>(Vector3d(0.0, 0.0, 3.0), Vector3d(0.0, 0.0, 0.0));
  plant.get_applied_spatial_force_input_port().FixValue(&context, forces);

  const double phi0 = -1.0e-3;  // Initial penetration into the ground, [m].
  plant.SetFreeBodyPose(&context, body,
                        RigidTransformd(Vector3d(0, 0, phi0)));

  // Visualize initial condition.
  driver.Publish();  // for viz.

  test::PgsSolver<double>& solver = driver.mutable_plant().set_contact_solver(
      std::make_unique<test::PgsSolver<double>>());

  // Specificaction of solver paramters happens here.
  test::PgsSolverParameters parameters;
  solver.set_parameters(parameters);

  auto params = driver.GetPointContactComplianceParameters(body);
  for (auto& p : params) {
    PRINT_VAR(p.first);
    PRINT_VAR(p.second);
  }

  driver.AdvanceNumSteps(1);

  PRINT_VAR(solver.GetImpulses().transpose());
  PRINT_VAR(solver.GetVelocities().transpose());
  PRINT_VAR(solver.GetContactVelocities().transpose());
  PRINT_VAR(solver.GetGeneralizedContactImpulses().transpose());

  auto& stats = solver.get_iteration_stats();
  PRINT_VAR(stats.iterations);
  PRINT_VAR(stats.vc_err);
  PRINT_VAR(stats.gamma_err);
  

  //const VectorX<double> v0 = VectorX<double>::Zero(nv);
  //driver.AdvanceOneStep(v0, dt);
  //driver.Publish();  // for viz.
}

#if 0

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
#endif

}  // namespace
}  // namespace solvers
}  // namespace multibody
}  // namespace drake

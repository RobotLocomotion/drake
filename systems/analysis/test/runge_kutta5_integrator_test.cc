#include "drake/systems/analysis/runge_kutta5_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/quartic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/quintic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

typedef ::testing::Types<RungeKutta5Integrator<double>> Types;
INSTANTIATE_TYPED_TEST_CASE_P(My, ExplicitErrorControlledIntegratorTest, Types);
INSTANTIATE_TYPED_TEST_CASE_P(My, PleidesTest, Types);

// A testing fixture for RK5 integrators, providing a simple
// free body plant with closed form solutions to test against.
class RK5IntegratorTest : public ::testing::Test {
 protected:
  void SetUp() {
    plant_ = std::make_unique<multibody::MultibodyPlant<double>>();

    // Add a single free body to the world.
    const double radius = 0.05;   // m
    const double mass = 0.1;      // kg
    auto G_Bcm = multibody::UnitInertia<double>::SolidSphere(radius);
    multibody::SpatialInertia<double> M_Bcm(
      mass, Vector3<double>::Zero(), G_Bcm);
    plant_->AddRigidBody("Ball", M_Bcm);
    plant_->Finalize();
  }

  std::unique_ptr<Context<double>> MakePlantContext() const {
    std::unique_ptr<Context<double>> context =
        plant_->CreateDefaultContext();
    context->EnableCaching();

    // Set body linear and angular velocity.
    Vector3<double> v0(1., 2., 3.);    // Linear velocity in body's frame.
    Vector3<double> w0(-4., 5., -6.);  // Angular velocity in body's frame.
    VectorX<double> generalized_velocities(6);
    generalized_velocities << w0, v0;
    plant_->SetVelocities(context.get(), generalized_velocities);

    // Set body position and orientation.
    Vector3<double> p0(1., 2., 3.);  // Body's frame position in the world.
    // Set body's frame orientation to 90 degree rotation about y-axis.
    Vector4<double> q0(std::sqrt(2.)/2., 0., std::sqrt(2.)/2., 0.);
    VectorX<double> generalized_positions(7);
    generalized_positions << q0, p0;
    plant_->SetPositions(context.get(), generalized_positions);

    return context;
  }

  std::unique_ptr<multibody::MultibodyPlant<double>> plant_{};
};

// Tests accuracy for integrating the quintic system (with the state at time t
// corresponding to f(t) ≡ t⁵ + 2t⁴ + 3t³ + 4t² + 5t + C) over t ∈ [0, 1].
// RK5 is a fifth order integrator, meaning that it uses the Taylor Series
// expansion: f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + ⅙h³f'''(t) + ... + h⁵/120
// f'''''(t) + O(h⁶). The formula above indicates that the approximation error
// will be zero if d⁶f/dt⁶ = 0, which is true for the quintic equation.
GTEST_TEST(RK5IntegratorErrorEstimatorTest, QuinticTest) {
  QuinticScalarSystem quintic;
  auto quintic_context = quintic.CreateDefaultContext();
  const double C = quintic.Evaluate(0);
  quintic_context->SetTime(0.0);
  quintic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta5Integrator<double> rk5(quintic, quintic_context.get());
  const double t_final = 1.0;
  rk5.set_maximum_step_size(t_final);
  rk5.set_fixed_step_mode(true);
  rk5.Initialize();
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final));

  // Check for near-exact 5th-order results. The measure of accuracy is a
  // tolerance that scales with expected answer at t_final.
  const double expected_answer =
      t_final * (t_final * (t_final * (t_final * (t_final + 2) + 3) + 4) + 5) +
      6;
  const double allowable_5th_order_error =
      expected_answer * std::numeric_limits<double>::epsilon();
  const double actual_answer =
      quintic_context->get_continuous_state_vector()[0];
  EXPECT_NEAR(actual_answer, expected_answer, allowable_5th_order_error);

  // This integrator calculates error by subtracting a 4th-order integration
  // result from a 5th-order integration result. Since the 4th-order integrator
  // has a Taylor series that is accurate to O(h⁵), halving the step size is
  // associated with an error that reduces by a factor of 2⁵ = 32.
  // So we verify that the error associated with a half-step is nearly 1/32 the
  // error associated with a full step.

  // First obtain the error estimate using a single step of h.
  const double err_est_h = rk5.get_error_estimate()->get_vector().GetAtIndex(0);

  // Now obtain the error estimate using two half steps of h/2.
  quintic_context->SetTime(0.0);
  quintic_context->get_mutable_continuous_state_vector()[0] = C;
  rk5.Initialize();
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final / 2));
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final));
  const double err_est_2h_2 =
      rk5.get_error_estimate()->get_vector().GetAtIndex(0);

  // Check that the 4th-order error estimate for a full-step is less than
  // K*32 times larger than then 4th-order error estimate for 2 half-steps;
  // K is a constant term that subsumes the asymptotic error and is dependent
  // upon both the system being integrated and the particular integrator.
  const double K = 128;
  const double allowable_4th_order_error =
      K * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(err_est_h, 32 * err_est_2h_2, allowable_4th_order_error);
}

// Tests accuracy for integrating the quartic system (with the state at time t
// corresponding to f(t) ≡ t⁴ + 2t³ + 3t² + 4t + C, where C is the initial
// state) over t ∈ [0, 1]. The error estimator from RK5 is fourth order, meaning
// that it uses the Taylor Series expansion: f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t)
// + ... + h⁴/24 f''''(t) + O(h⁵). This formula indicates that the approximation
// error will be zero if d⁵f/dt⁵ = 0, which is true for the quartic equation. We
// check that the error estimator gives a perfect error estimate for this
// function.
GTEST_TEST(RK5IntegratorErrorEstimatorTest, QuarticTest) {
  QuarticScalarSystem quartic;
  auto quartic_context = quartic.CreateDefaultContext();
  const double C = quartic.Evaluate(0);
  quartic_context->SetTime(0.0);
  quartic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta5Integrator<double> rk5(quartic, quartic_context.get());
  const double t_final = 1.0;
  rk5.set_maximum_step_size(t_final);
  rk5.set_fixed_step_mode(true);
  rk5.Initialize();
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est = rk5.get_error_estimate()->get_vector().GetAtIndex(0);

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());
}

// Tests accuracy of integrator's dense output.
TEST_F(RK5IntegratorTest, DenseOutputAccuracy) {
  std::unique_ptr<Context<double>> context = MakePlantContext();

  RungeKutta5Integrator<double> rk5(*plant_, context.get());
  rk5.set_maximum_step_size(0.1);
  rk5.set_target_accuracy(1e-6);
  rk5.Initialize();

  // Start a dense integration i.e. one that generates a dense
  // output for the state function.
  rk5.StartDenseIntegration();

  const double t_final = 1.0;
  // Arbitrary step, valid as long as it doesn't match the same
  // steps taken by the integrator. Otherwise, dense output accuracy
  // would not be checked.
  const double dt = 0.01;
  const int n_steps = (t_final / dt);
  for (int i = 1; i <= n_steps; ++i) {
    // Integrate the whole step.
    rk5.IntegrateWithMultipleStepsToTime(i * dt);

    // Check solution.
    EXPECT_TRUE(CompareMatrices(
        rk5.get_dense_output()->Evaluate(context->get_time()),
        plant_->GetPositionsAndVelocities(*context),
        rk5.get_accuracy_in_use(),
        MatrixCompareType::relative));
  }

  // Stop undergoing dense integration.
  std::unique_ptr<DenseOutput<double>> rk5_dense_output =
      rk5.StopDenseIntegration();
  EXPECT_FALSE(rk5.get_dense_output());

  // Integrate one more step.
  rk5.IntegrateWithMultipleStepsToTime(t_final + dt);

  // Verify that the dense output was not updated.
  EXPECT_LT(rk5_dense_output->end_time(), context->get_time());
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

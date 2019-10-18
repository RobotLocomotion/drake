#include "drake/systems/analysis/runge_kutta3_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

typedef ::testing::Types<RungeKutta3Integrator<double>> Types;
INSTANTIATE_TYPED_TEST_CASE_P(My, ExplicitErrorControlledIntegratorTest, Types);
INSTANTIATE_TYPED_TEST_CASE_P(My, PleidesTest, Types);

// A testing fixture for RK3 integrators, providing a simple
// free body plant with closed form solutions to test against.
class RK3IntegratorTest : public ::testing::Test {
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

// Tests accuracy for integrating the cubic system (with the state at time t
// corresponding to f(t) ≡ t³ + t² + 12t + C) over t ∈ [0, 1]. RK3 is a third
// order integrator, meaning that it uses the Taylor Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + ⅙h³f'''(t) + O(h⁴)
// The formula above indicates that the approximation error will be zero if
// f''''(t) = 0, which is true for the cubic equation.
GTEST_TEST(RK3IntegratorErrorEstimatorTest, CubicTest) {
  CubicScalarSystem cubic;
  auto cubic_context = cubic.CreateDefaultContext();
  const double C = cubic.Evaluate(0);
  cubic_context->SetTime(0.0);
  cubic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta3Integrator<double> rk3(cubic, cubic_context.get());
  const double t_final = 1.0;
  rk3.set_maximum_step_size(t_final);
  rk3.set_fixed_step_mode(true);
  rk3.Initialize();
  ASSERT_TRUE(rk3.IntegrateWithSingleFixedStepToTime(t_final));

  // Check for near-exact 3rd-order results. The measure of accuracy is a
  // tolerance that scales with expected answer at t_final.
  const double expected_answer = t_final * (t_final * (t_final + 1) + 12) + C;
  const double allowable_3rd_order_error = expected_answer *
      std::numeric_limits<double>::epsilon();
  const double actual_answer = cubic_context->get_continuous_state_vector()[0];
  EXPECT_NEAR(actual_answer, expected_answer, allowable_3rd_order_error);

  // This integrator calculates error by subtracting a 2nd-order integration
  // result from a 3rd-order integration result. Since the 2nd-order integrator
  // has a Taylor series that is accurate to O(h³), halving the step size is
  // associated with an error that reduces by a factor of 2³ = 8.
  // So we verify that the error associated with a half-step is nearly 1/8 the
  // error associated with a full step.

  // First obtain the error estimate using a single step of h.
  const double err_est_h =
      rk3.get_error_estimate()->get_vector().GetAtIndex(0);

  // Now obtain the error estimate using two half steps of h/2.
  cubic_context->SetTime(0.0);
  cubic_context->get_mutable_continuous_state_vector()[0] = C;
  rk3.Initialize();
  ASSERT_TRUE(rk3.IntegrateWithSingleFixedStepToTime(t_final/2));
  ASSERT_TRUE(rk3.IntegrateWithSingleFixedStepToTime(t_final));
  const double err_est_2h_2 =
      rk3.get_error_estimate()->get_vector().GetAtIndex(0);

  // Check that the 2nd-order error estimate for a full-step is less than
  // K*8 times larger than then 2nd-order error estimate for 2 half-steps;
  // K is a constant term that subsumes the asymptotic error and is dependent
  // upon both the system being integrated and the particular integrator.
  const double K = 256;
  const double allowable_2nd_order_error = K *
      std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(err_est_h, 8 * err_est_2h_2, allowable_2nd_order_error);
}

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ 4t² + 4t + C, where C is the initial state) over
// t ∈ [0, 1]. The error estimator from RK3 is
// second order, meaning that it uses the Taylor Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + O(h³)
// This formula indicates that the approximation error will be zero if
// f'''(t) = 0, which is true for the quadratic equation. We check that the
// error estimator gives a perfect error estimate for this function.
GTEST_TEST(RK3IntegratorErrorEstimatorTest, QuadraticTest) {
  QuadraticScalarSystem quadratic;
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C = quadratic.Evaluate(0);
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta3Integrator<double> rk3(quadratic, quadratic_context.get());
  const double t_final = 1.0;
  rk3.set_maximum_step_size(t_final);
  rk3.set_fixed_step_mode(true);
  rk3.Initialize();
  ASSERT_TRUE(rk3.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est =
      rk3.get_error_estimate()->get_vector().GetAtIndex(0);

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());
}

// Tests accuracy of integrator's dense output.
TEST_F(RK3IntegratorTest, DenseOutputAccuracy) {
  std::unique_ptr<Context<double>> context = MakePlantContext();

  RungeKutta3Integrator<double> rk3(*plant_, context.get());
  rk3.set_maximum_step_size(0.1);
  rk3.set_target_accuracy(1e-6);
  rk3.Initialize();

  // Start a dense integration i.e. one that generates a dense
  // output for the state function.
  rk3.StartDenseIntegration();

  const double t_final = 1.0;
  // Arbitrary step, valid as long as it doesn't match the same
  // steps taken by the integrator. Otherwise, dense output accuracy
  // would not be checked.
  const double dt = 0.01;
  const int n_steps = t_final / dt;
  for (int i = 1; i < n_steps; ++i) {
    // Integrate the whole step.
    rk3.IntegrateWithMultipleStepsToTime(i * dt);

    // Check solution.
    EXPECT_TRUE(CompareMatrices(
        rk3.get_dense_output()->Evaluate(context->get_time()),
        plant_->GetPositionsAndVelocities(*context),
        rk3.get_accuracy_in_use(),
        MatrixCompareType::relative));
  }

  // Stop undergoing dense integration.
  std::unique_ptr<DenseOutput<double>> rk3_dense_output =
      rk3.StopDenseIntegration();
  EXPECT_FALSE(rk3.get_dense_output());

  // Integrate one more step.
  rk3.IntegrateWithMultipleStepsToTime(t_final);

  // Verify that the dense output was not updated.
  EXPECT_LT(rk3_dense_output->end_time(), context->get_time());
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

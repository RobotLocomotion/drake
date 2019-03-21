#include "drake/systems/analysis/runge_kutta3_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"

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

// System where the state at t corresponds to the cubic equation
// t³ + t² + 12t + C, where C is the initial value (the state at t=0).
class Cubic : public LeafSystem<double> {
 public:
  Cubic() { this->DeclareContinuousState(1); }

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* deriv) const override {
    const double t = context.get_time();
    (*deriv)[0] = 3 * t * t + 2 * t + 12;
  }
};

// System where the state at t corresponds to the quadratic equation
// 4t² + 4t + C, where C is the initial value (the state at t=0).
class Quadratic : public LeafSystem<double> {
 public:
  Quadratic() { this->DeclareContinuousState(1); }

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* deriv) const override {
    const double t = context.get_time();
    (*deriv)[0] = 8 * t + 4;
  }
};

// Tests accuracy for integrating the cubic system (with the state at time t
// corresponding to f(t) ≡ t³ + t² + 12t + C) over
// t ∈ [0, 1]. RK3 is a third order integrator, meaning that it uses the Taylor
// Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + ⅙h³f'''(t) + O(h⁴)
// The formula above indicates that the approximation error will be zero if
// f''''(t) = 0, which is true for the cubic equation.
GTEST_TEST(RK3IntegratorErrorEstimatorTest, CubicTest) {
  Cubic cubic;
  auto cubic_context = cubic.CreateDefaultContext();
  const double C = 0.0;
  cubic_context->set_time(0.0);
  cubic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta3Integrator<double> rk3(cubic, cubic_context.get());
  const double t_final = 1.0;
  rk3.set_maximum_step_size(t_final);
  rk3.set_fixed_step_mode(true);
  rk3.Initialize();
  rk3.IntegrateWithSingleFixedStepToTime(t_final);

  const double expected_answer = t_final * (t_final * (t_final + 1) + 12);
  EXPECT_NEAR(
      cubic_context->get_continuous_state_vector()[0], expected_answer,
      10 * std::numeric_limits<double>::epsilon());

  // Now verify that the error in the second-order error estimate shrinks by a
  // factor of eight, as the Taylor Series formula would predict. Note that the
  // true error is zero.

  // First obtain the error estimate using a single step of h.
  const double err_est_h =
      rk3.get_error_estimate()->get_vector().GetAtIndex(0);

  // Now obtain the error estimate using two half steps of h/2.
  cubic_context->set_time(0.0);
  cubic_context->get_mutable_continuous_state_vector()[0] = C;
  rk3.Initialize();
  rk3.IntegrateWithSingleFixedStepToTime(t_final/2);
  rk3.IntegrateWithSingleFixedStepToTime(t_final);
  const double err_est_2h_2 =
      rk3.get_error_estimate()->get_vector().GetAtIndex(0);

  // The second order estimate should be approximately 8 times smaller.
  EXPECT_NEAR(err_est_2h_2 * 8, err_est_h,
      250 * std::numeric_limits<double>::epsilon());
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
  Quadratic quadratic;
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C = 0.0;
  quadratic_context->set_time(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta3Integrator<double> rk3(quadratic, quadratic_context.get());
  const double t_final = 1.0;
  rk3.set_maximum_step_size(t_final);
  rk3.set_fixed_step_mode(true);
  rk3.Initialize();
  rk3.IntegrateWithSingleFixedStepToTime(t_final);

  const double err_est =
      rk3.get_error_estimate()->get_vector().GetAtIndex(0);
  EXPECT_NEAR(err_est, 0.0, std::numeric_limits<double>::epsilon());
}

// Tests accuracy when generalized velocity is not the time derivative of
// generalized configuration against an RK2 integrator.
TEST_F(RK3IntegratorTest, ComparisonWithRK2) {
  // Integrate for ten thousand steps using a RK2 integrator with
  // small step size.
  const double dt = 5e-5;
  std::unique_ptr<Context<double>> rk2_context = MakePlantContext();
  RungeKutta2Integrator<double> rk2(*plant_, dt, rk2_context.get());

  rk2.Initialize();
  const double t_final = 1.0;
  const int n_steps = t_final / dt;
  for (int i = 1; i <= n_steps; ++i)
    rk2.IntegrateWithSingleFixedStepToTime(i * dt);

  // Re-integrate with RK3.
  std::unique_ptr<Context<double>> rk3_context = MakePlantContext();
  RungeKutta3Integrator<double> rk3(*plant_, rk3_context.get());
  rk3.set_maximum_step_size(0.1);
  rk3.set_target_accuracy(1e-6);
  rk3.Initialize();

  // Verify that IntegrateWithMultipleSteps works.
  const double tol = std::numeric_limits<double>::epsilon();
  rk3.IntegrateWithMultipleStepsToTime(t_final);
  EXPECT_NEAR(rk3_context->get_time(), t_final, tol);

  // Verify that the final states are "close".
  const VectorBase<double>& x_final_rk2 =
      rk2_context->get_continuous_state_vector();

  const VectorBase<double>& x_final_rk3 =
      rk3_context->get_continuous_state_vector();

  const double close_tol = 2e-6;
  for (int i = 0; i < x_final_rk2.size(); ++i)
    EXPECT_NEAR(x_final_rk2[i], x_final_rk3[i], close_tol);
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

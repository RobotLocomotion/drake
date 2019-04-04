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

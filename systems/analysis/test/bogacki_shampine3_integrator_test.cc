#include "drake/systems/analysis/bogacki_shampine3_integrator.h"

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

typedef ::testing::Types<BogackiShampine3Integrator<double>> Types;
INSTANTIATE_TYPED_TEST_CASE_P(My, ExplicitErrorControlledIntegratorTest, Types);
INSTANTIATE_TYPED_TEST_CASE_P(My, PleidesTest, Types);

// A testing fixture for the BS3 integrator, providing a simple
// free body plant with closed form solutions to test against.
class BS3IntegratorTest : public ::testing::Test {
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
// corresponding to f(t) ≡ t³ + t² + 12t + C) over t ∈ [0, 1]. BS3 is a third
// order integrator, meaning that it uses the Taylor Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + ⅙h³f'''(t) + O(h⁴)
// The formula above indicates that the approximation error will be zero if
// f''''(t) = 0, which is true for the cubic equation.
GTEST_TEST(BS3IntegratorErrorEstimatorTest, CubicTest) {
  CubicScalarSystem cubic;
  auto cubic_context = cubic.CreateDefaultContext();
  const double C = cubic.Evaluate(0);
  cubic_context->SetTime(0.0);
  cubic_context->get_mutable_continuous_state_vector()[0] = C;

  BogackiShampine3Integrator<double> bs3(cubic, cubic_context.get());
  const double t_final = 1.0;
  bs3.set_maximum_step_size(t_final);
  bs3.set_fixed_step_mode(true);
  bs3.Initialize();
  ASSERT_TRUE(bs3.IntegrateWithSingleFixedStepToTime(t_final));

  // Check for near-exact 3rd-order results. The measure of accuracy is a
  // tolerance that scales with expected answer at t_final.
  const double expected_answer = cubic.Evaluate(t_final);
  const double allowable_3rd_order_error = expected_answer *
      std::numeric_limits<double>::epsilon();
  const double actual_answer = cubic_context->get_continuous_state_vector()[0];
  EXPECT_NEAR(actual_answer, expected_answer, allowable_3rd_order_error);
}

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ 4t² + 4t + C, where C is the initial state) over
// t ∈ [0, 1]. The error estimate from BS3 is second order accurate, meaning
// that the approximation error will be zero if f'''(t) = 0, which is true for
// the quadratic equation. We check that the error estimate is perfect for this
// function.
GTEST_TEST(BS3IntegratorErrorEstimatorTest, QuadraticTest) {
  QuadraticScalarSystem quadratic;
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C = quadratic.Evaluate(0);
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;

  BogackiShampine3Integrator<double> bs3(quadratic, quadratic_context.get());
  const double t_final = 1.0;
  bs3.set_maximum_step_size(t_final);
  bs3.set_fixed_step_mode(true);
  bs3.Initialize();
  ASSERT_TRUE(bs3.IntegrateWithSingleFixedStepToTime(t_final));

  // Per the description in IntegratorBase::get_error_estimate_order(), this
  // should return "3", in accordance with the order of the polynomial in the
  // Big-Oh term.
  ASSERT_EQ(bs3.get_error_estimate_order(), 3);

  const double err_est =
      bs3.get_error_estimate()->get_vector().GetAtIndex(0);

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

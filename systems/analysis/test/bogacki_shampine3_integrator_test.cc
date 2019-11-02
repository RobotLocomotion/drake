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
#include "drake/systems/analysis/test_utilities/generic_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

typedef ::testing::Types<BogackiShampine3Integrator<double>> Types;
// NOLINTNEXTLINE(whitespace/line_length)
INSTANTIATE_TYPED_TEST_SUITE_P(My, ExplicitErrorControlledIntegratorTest, Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, PleidesTest, Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, GenericIntegratorTest, Types);

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

  // This integrator calculates error by subtracting a 2nd-order integration
  // result from a 3rd-order integration result. Since the 2nd-order integrator
  // has a Taylor series that is accurate to O(h³) and since we have no terms
  // beyond order h³, halving the step size should improve the error estimate
  // by a factor of 2³ = 8. We verify this.

  // First obtain the error estimate using a single step of h.
  const double err_est_h = bs3.get_error_estimate()->get_vector().GetAtIndex(0);

  // Now obtain the error estimate using two half steps of h/2.
  cubic_context->SetTime(0.0);
  cubic_context->get_mutable_continuous_state_vector()[0] = C;
  bs3.Initialize();
  ASSERT_TRUE(bs3.IntegrateWithSingleFixedStepToTime(t_final / 2));
  ASSERT_TRUE(bs3.IntegrateWithSingleFixedStepToTime(t_final));
  const double err_est_2h_2 =
      bs3.get_error_estimate()->get_vector().GetAtIndex(0);

  EXPECT_NEAR(err_est_2h_2, 1.0 / 8 * err_est_h,
              10 * std::numeric_limits<double>::epsilon());
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

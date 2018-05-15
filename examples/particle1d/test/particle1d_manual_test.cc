#include "drake/examples/particle1d/particle1d_manual.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace examples {
namespace particle1d {
namespace {

// Helper function to test the expected value of the class parameter.
void VerifyValueOnly(double test_parameter, double value) {
  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(test_parameter, value, kEpsilon);
}

// Helper function to test the AutoDiff value and partial derivatives.
// Note: partial with respect to variable A (∂y/∂A) will be denoted as dy_dA.
void VerifyAutoDiffValueAndPartialDerivatives(const AutoDiffXd& y,
                                              double y_value, double dy_dA,
                                              double dy_dB, double dy_dC) {
  // First test the value of the AutoDiff evaluated at the value set in the
  // AutoDiff declaration.
  VerifyValueOnly(y.value(), y_value);

  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();

  // Test ∂y/∂A.
  EXPECT_NEAR(y.derivatives()(0), dy_dA, kEpsilon);

  // Test ∂y/∂B.
  EXPECT_NEAR(y.derivatives()(1), dy_dB, kEpsilon);

  // Test ∂y/∂C.
  EXPECT_NEAR(y.derivatives()(2), dy_dC, kEpsilon);
}

// Unit test for the Particle1dManual class with double instantiation.
GTEST_TEST(PlantTest, ManualClassTestDouble) {
  Particle1dManual<double> test_particle;

  Particle1dManual<double>::ParticleData& particle_data =
      test_particle.get_particle_data();

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(particle_data.mass_ == 1);

  // Struct for the test data.
  struct TestData {
    double test_time;
    double test_state[2];
  };

  // Test data for three separate test cases.
  std::vector<TestData> test_data(3);
  test_data[0] = {0.0, {0.0, 0.0}};
  test_data[1] = {0.75, {0.26831113112, 0.68163876002}};
  test_data[2] = {0.9, {0.37839003172, 0.78332690962}};

  // For-loop to check three separate cases.
  for (int i = 0; i < 3; i++) {
    double stateDt[2];

    // Calculate state derivatives and test their values against expected,
    // analytical solutions.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    test_particle.CalcDerivativesToStateDt(test_data[i].test_time,
                                           test_data[i].test_state, stateDt);

    VerifyValueOnly(stateDt[0],
                    test_data[i].test_state[1]);  // ẋ = v = state[1]
    VerifyValueOnly(stateDt[1],
                    std::cos(test_data[i].test_time) /
                        particle_data.mass_);  // ẍ = cos(time)/m
  }
}

// Unit test for the Particle1dManual class with AutoDiffXd instantiation.
// Note: This unit test only checks AutoDiff values (not derivatives).
// The next test is more complicated as it also checks partial derivatives.
GTEST_TEST(PlantTest, ManualClassTestAutoDiffValuesOnly) {
  Particle1dManual<AutoDiffXd> test_particle;

  Particle1dManual<AutoDiffXd>::ParticleData& particle_data =
      test_particle.get_particle_data();

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(particle_data.mass_ == 1);

  // Struct for the test data.
  struct TestData {
    AutoDiffXd test_time;
    AutoDiffXd test_state[2];
  };

  // Test data for three separate test cases.
  std::vector<TestData> test_data(3);
  test_data[0] = {0.0, {0.0, 0.0}};
  test_data[1] = {0.75, {0.26831113112, 0.68163876002}};
  test_data[2] = {0.9, {0.37839003172, 0.78332690962}};

  // for loop to check the repeated statements
  for (int i = 0; i < 3; i++) {
    AutoDiffXd stateDt[2];

    // Calculate state derivatives and test their values against expected,
    // analytical solutions.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    test_particle.CalcDerivativesToStateDt(test_data[i].test_time,
                                           test_data[i].test_state, stateDt);

    VerifyValueOnly(stateDt[0].value(),
                    test_data[i].test_state[1].value());  // ẋ = v = state[1]
    VerifyValueOnly(stateDt[1].value(),
                    cos(test_data[i].test_time.value()) /
                        particle_data.mass_.value());  // ẍ = cos(time)/m
  }
}

// Unit test for the Particle1dManual class with AutoDiffXd instantiation.
// Note: This unit test is more complicated than the previous unit test as it
// checks partial derivative calculations (whereas the previous test only checks
// AutoDiff values).
GTEST_TEST(PlantTest, AutoDiffPartials) {
  Particle1dManual<AutoDiffXd> test_particle;

  const double mass_value = 3.0;
  const double time_value = 2.0;
  const double x_value = 0.5;
  const double xDt_value = 0.7;

  // Initialize the AutoDiff with arbitrary values at which the derivative will
  // be evaluated. Note: A more explicit, pedagogical example for how to set up
  // an AutoDiff can be found in autodiff_test.cc.
  Eigen::Vector3d auto_values;
  auto_values[0] = mass_value;  // Variable mass
  auto_values[1] = time_value;  // Variable time
  auto_values[2] = x_value;     // Variable x


  // Use an identity matrix to initialize the partials to:
  // ∂m/∂m = 1, ∂t/∂t = 1, ∂x/∂x = 1.
  Eigen::Matrix3d gradient_matrix;
  gradient_matrix.setIdentity();

  // Create and initialize an AutoDiff matrix given the values and
  // gradient matrix.
  auto autodiff_vars = drake::math::initializeAutoDiffGivenGradientMatrix(
      auto_values, gradient_matrix);

  // Get struct that contains the particles data (mass, state, stateDt, and F).
  Particle1dManual<AutoDiffXd>::ParticleData& particle_data =
      test_particle.get_particle_data();

  AutoDiffXd state[2];
  AutoDiffXd stateDt[2];

  particle_data.mass_ = autodiff_vars[0];  // autodiff_vars[0] is AutoDiff mass.
  AutoDiffXd time = autodiff_vars[1];      // autodiff_vars[1] is AutoDiff time.
  state[0] = autodiff_vars[2];             // autodiff_vars[2] is AutoDiff x.
  state[1].value() = xDt_value;            // AutoDiff ẋ.

  test_particle.CalcDerivativesToStateDt(time, state, stateDt);

  // Verify AutoDiff values from initialization.
  // Value: m = mass (assigned above).
  // The expected values are: m = 3, ∂m/∂m = 1, ∂m/∂t = 0, ∂m/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.mass_, mass_value, 1,
                                           0, 0);

  // Value: x = state[0] (assigned above).
  // The expected values are: x = 0.5, ∂x/∂m = 0, ∂x/∂t = 0, ∂x/∂x = 1.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.x_, x_value, 0, 0, 1);

  // Verify AutoDiff values from calculation.
  // Value: ẍ = cos(t)/m.
  // The expected values are: ẍ = cos(t)/m, ∂ẍ/∂m = -cos(t)/m²,
  // ∂ẍ/∂t = -sint(t)/m, ∂ẍ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(
      stateDt[1], cos(time.value()) / particle_data.mass_.value(),
      -cos(time.value()) /
          (particle_data.mass_.value() * particle_data.mass_.value()),
      -sin(time.value()) / particle_data.mass_.value(), 0);

  // Value: f = cos(t).
  // The expected values are: f = cos(t), ∂f/∂m = 0, ∂f/∂t = -sin(t), ∂f/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.F_, cos(time.value()),
                                           0, -sin(time.value()), 0);
}

}  // namespace
}  // namespace particle1d
}  // namespace examples
}  // namespace drake

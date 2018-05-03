#include "drake/examples/particle1d/MG/particle1dMG.h"

#include <gtest/gtest.h>

namespace drake {
namespace examples {
namespace particle1d {
namespace {

// Helper function to test the expected value of the class parameter.
void VerifyValueOnly(double test_parameter, double value) {
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(test_parameter, value, kEpsilon);
}

// Helper function to test the AutoDiff value and partial derivatives.
void VerifyAutoDiffValueAndPartialDerivatives(
    const AutoDiffXd& y, double y_value,
    double y_partial_with_respect_to_variableA,
    double y_partial_with_respect_to_variableB,
    double y_partial_with_respect_to_variableC) {

  // First test the value of the AutoDiff evaluated at the value set in the
  // AutoDiff declaration.
  VerifyValueOnly(y.value(), y_value);

  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

  // Test ∂y/∂A (partial with respect to variable A).
  EXPECT_NEAR(y.derivatives()(0), y_partial_with_respect_to_variableA,
              kEpsilon);

  // Test ∂y/∂B (partial with respect to variable B).
  EXPECT_NEAR(y.derivatives()(1), y_partial_with_respect_to_variableB,
              kEpsilon);
}

// Unit test for the Particle1dMG class with Double instantiation.
GTEST_TEST(PlantTest, MGClassTestDouble) {
  MotionGenesis::Particle1dMG<double> test_particle;

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(test_particle.mass == 1);

  // For-loop to check three separate cases.
  for (int i = 0; i < 3; i++) {
  double time = 0.0;
  double state[2] = {0};

    if (i == 1) {
      // Test calculations for another value of time and another state.
      test_particle.mass = 1;
      time = 0.75;
      state[0] = 0.26831113112;
      state[1] = 0.68163876002;
    }else if(i== 2){
      // Test calculations for another value of time and another state.
      test_particle.mass = 25;
      time = 0.9;
      state[0] = 0.37839003172;
      state[1] = 0.78332690962;
    }

    // Ensure time-derivative of state is properly calculated.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    double stateDt[2];

    // Calculate state derivatives and test their values against expected,
    // analytical solutions.
    test_particle.CalcDerivativesToStateDt(time, state, stateDt);

    VerifyValueOnly(stateDt[0], state[1]);                 // ẋ = ẋ
    VerifyValueOnly(stateDt[1],
                    std::cos(time) / test_particle.mass);  // ẍ = cos(time)/m
  }
}

// Unit test for the Particle1dMG class with AutoDiffXd instantiation.
// Note: This unit test only checks AutoDiff values (not derivatives).
// The next test is more complicated as it also checks partial derivatives.
GTEST_TEST(PlantTest, MGClassTestAutoDiffValuesOnly) {
  MotionGenesis::Particle1dMG<AutoDiffXd> test_particle;

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(test_particle.mass == 1);

  // for loop to check the repeated statements
  for (int i = 0; i < 3; i++) {
    test_particle.mass = 1;
    AutoDiffXd time = 0.0;
    AutoDiffXd state[2] = {0};

    if (i == 1) {
      // Test calculations for another value of time and another state.
      test_particle.mass = 1;
      time = 0.75;
      state[0] = 0.26831113112;
      state[1] = 0.68163876002;
    } else if (i == 2) {
      // Test calculations for another value of time and another state.
      test_particle.mass = 25;
      time = 0.9;
      state[0] = 0.37839003172;
      state[1] = 0.78332690962;
    }

    // Ensure time-derivative of state is properly calculated for each of the
    // three cases.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    AutoDiffXd stateDt[2];

    // Calculate state derivatives and test their values against expected,
    // analytical solutions.
    test_particle.CalcDerivativesToStateDt(time, state, stateDt);

    VerifyValueOnly(stateDt[0].value(), state[1].value()); // ẋ = ẋ
    VerifyValueOnly(stateDt[1].value(),
        cos(time.value()) / test_particle.mass.value());  // ẍ = cos(time)/m
  }
}

// Unit test for the Particle1dMG class with AutoDiffXd instantiation.
// Note: This unit test is more complicated than the previous unit test as it
// checks partial derivative calculations (whereas the previous test only checks
// AutoDiff values).
GTEST_TEST(PlantTest, MGClassTestAutoDiffDerivativeTest) {
  MotionGenesis::Particle1dMG<AutoDiffXd> test_particle;
  // Calculate partial derivatives with respect to 2 variables (time and x).
  const int num_variables_differentiate_with_respect_to = 3;
  const int index_for_partial_mass = 0;
  const int index_for_partial_time = 1;
  const int index_for_partial_x = 2;

  // Set the value of time at which the partials will be evaluated.
  // Arbitrarily chose time = 2 seconds.
  AutoDiffXd time;
  time.value() = 2.0;

  // Provide enough room to store partials with respect to 3 variables:
  // mass, time, and x
  time.derivatives().resize(num_variables_differentiate_with_respect_to);

  // Set the partial of time with respect to mass to 0 (∂t/∂m = 0).
  // Set the partial of time with respect to time to 1 (∂t/∂t = 1).
  // Set the partial of time with respect to x to 0 (∂t/∂x = 0).
  time.derivatives()(index_for_partial_mass) = 0.0;
  time.derivatives()(index_for_partial_time) = 1.0;
  time.derivatives()(index_for_partial_x) = 0.0;

  // Set the value of the particle's mass. Arbitrarily chosen to be 20 kg.
  AutoDiffXd& mass = test_particle.mass;
  mass.value() = 20.0;

  // Provide enough room to store partials with respect to 3 variables:
  // mass, time, and x
  mass.derivatives().resize(num_variables_differentiate_with_respect_to);

  // Set the partial of mass with respect to mass to 1 (∂m/∂m = 1).
  // Set the partial of mass with respect to time to 0 (∂m/∂t = 0).
  // Set the partial of mass with respect to x to 0 (∂m/∂x = 0).
  mass.derivatives()(index_for_partial_mass) = 1.0;
  mass.derivatives()(index_for_partial_time) = 0.0;
  mass.derivatives()(index_for_partial_x) = 0.0;

  // The state vector has the following assignments:
  // state[0] = x and state[1] = xDt.
  // Set the value of the variables (state[0] and state[1]) to the value at
  // which the derivative is to be evaluated.  Arbitrarily chose [0.5, 0.7].
  AutoDiffXd state[2];
  state[0].value() = 0.5;
  state[1].value() = 0.7;

  // Provide enough room to store partials with respect to 3 variables:
  // mass, time, and x
  state[0].derivatives().resize(num_variables_differentiate_with_respect_to);
  state[1].derivatives().resize(num_variables_differentiate_with_respect_to);

  // Set the partial of state[0] with respect to mass to 0 (∂x/∂m = 0).
  // Set the partial of state[0] with respect to time to 0 (∂x/∂t = 0).
  // Set the partial of state[0] with respect to x (itself) to 1 (∂x/∂x = 1).
  state[0].derivatives()(index_for_partial_mass) = 1.0;
  state[0].derivatives()(index_for_partial_time) = 0.0;
  state[0].derivatives()(index_for_partial_x) = 1.0;

  // Set the partial of state[1] with respect to mass to 0 (∂ẋ/∂m = 0).
  // Set the partial of state[1] with respect to time to 0 (∂ẋ/∂t = 0).
  // Set the partial of state[1] with respect to x (itself) to 0 (∂ẋ/∂x = 0).
  state[1].derivatives()(index_for_partial_mass) = 0.0;
  state[1].derivatives()(index_for_partial_time) = 0.0;
  state[1].derivatives()(index_for_partial_x) = 0.0;

  // stateDt is calculated, hence its value and derivatives will be assigned.
  AutoDiffXd stateDt[2];
  test_particle.CalcDerivativesToStateDt(time, state, stateDt);

  // Value: m = mass (assigned above).
  // The expected values are: ∂m/∂m = 1, ∂m/∂t = 0, ∂m/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.mass, mass.value(), 1,
                                           0, 0);

  // Value: x = state[0] (assigned above).
  // The expected values are: ∂x/∂m = 0, ∂x/∂t = 0, ∂x/∂x = 1.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.x, state[0].value(), 1,
                                           0, 1);

  // Value: ẋ = state[1] (assigned above).
  // The expected values are: ∂ẋ/∂m = 0, ∂ẋ/∂t = 0, ∂ẋ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.xDt, state[1].value(),
                                           0, 0, 0);

  // Value: ẍ = cos(t)/m.
  // The expected values are: ∂ẍ/∂m = -cos(t)/m², ∂ẍ/∂t = -sint(t)/m, ∂ẍ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(
      test_particle.xDDt, cos(time.value()) / mass.value(),
      -cos(time.value()) / (mass.value() * mass.value()),
      -sin(time.value()) / mass.value(), 0);

  // Value: f = cos(t).
  // The expected values are: ∂f/∂m = 0, ∂f/∂t = -sin(t), ∂f/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.F, cos(time.value()),
                                           0, -sin(time.value()), 0);
}

} // namespace
} // namespace particle1d
} // namespace examples
} // namespace drake

#include "drake/examples/particle1d/MG/particle1dMG.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace examples {
namespace particle1d {
namespace {

// Helper function to test the expected value of the class parameter.
void VerifyValueOnly(double test_parameter, double value) {
  EXPECT_EQ(test_parameter, value);
}

// Helper function to test the AutoDiff value and partial derivatives.
void VerifyAutoDiffValueAndPartialDerivatives(
    const AutoDiffXd& y, double y_value,
    double y_partial_with_respect_to_variableA,
    double y_partial_with_respect_to_variableB) {

  // First test the value of the AutoDiff evaluated at the value set in the
  // AutoDiff declaration.
  VerifyValueOnly(y.value(), y_value);

  // Test partials.
  // Partial with respect to variable A.
  EXPECT_EQ(y.derivatives()(0), y_partial_with_respect_to_variableA);

  // Partial with respect to variable B.
  EXPECT_EQ(y.derivatives()(1), y_partial_with_respect_to_variableB);

}

// Unit test for the Particle1dMG class with Double instantiation.
GTEST_TEST(PlantTest, MGClassTestDouble) {
  MotionGenesis::Particle1dMG<double> test_particle;

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(test_particle.mass == 1);

  // for loop to check the repeated statements
  for (int i = 0; i < 3; i++) {
  double time = 0.0;
  double state[2] = {0};

    if (i == 1) {
      // Test calculations for another value of time and another state.
      test_particle.mass = 1; // mass to 1 kg
      time = 0.75;
      state[0] = 0.26831113112;
      state[1] = 0.68163876002;
    }else if(i== 2){
      // Test calculations for another value of time and another state.
      test_particle.mass = 25; // mass to 25 kg
      time = 0.9;
      state[0] = 0.37839003172;
      state[1] = 0.78332690962;
    }

    // Ensure time-derivative of state is properly calculated.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    //double tolerance = 1e-300;
    double stateDt[2] = {0};

    // Calculate state derivatives and test their values against expected,
    // analytical solutions.
    test_particle.CalcDerivativesToStateDt(time, state, stateDt);
    VerifyValueOnly(stateDt[0], state[1]);  // ẋ = ẋ
    VerifyValueOnly(stateDt[1], cos(time) / test_particle.mass);  // ẍ = cos(time)/m
  }
}

// Unit test for the Particle1dMG class with AutoDiffXd instantiation.
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
      test_particle.mass = 25;  // mass to 25 kg
      time = 0.9;
      state[0] = 0.37839003172;
      state[1] = 0.78332690962;
    }

    // Ensure time-derivative of state is properly calculated when state has
    // initial values of zero.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    AutoDiffXd stateDt[2] = {0};

    // Calculate state derivatives and test their values against expected,
    // analytical solutions.
    test_particle.CalcDerivativesToStateDt(time, state, stateDt);
    VerifyValueOnly(stateDt[0].value(), state[1].value());  // ẋ = ẋ
    VerifyValueOnly(stateDt[1].value(),
        cos(time.value()) / test_particle.mass.value());  // ẍ = cos(time)/m
 }
}

// Unit test for the Particle1dMG class with AutoDiffXd instantiation.
GTEST_TEST(PlantTest, MGClassTestAutoDiffDerivativeTest) {
  MotionGenesis::Particle1dMG<AutoDiffXd> test_particle;
  // Calculate partial derivatives with respect to 2 variables (time and x).
  const int num_variables_differentiate_with_respect_to = 2;
  const int index_for_partial_time = 0;
  const int index_for_partial_x = 1;

  // Set up partial differentiation with respect to time.
  AutoDiffXd test_time;
  test_time.value() = 2.0;
  test_time.derivatives().resize(num_variables_differentiate_with_respect_to);
  test_time.derivatives()(index_for_partial_time) = 1.0;
  test_time.derivatives()(index_for_partial_x) = 0.0;

  // Set up partial differentiation with respect to time.
  AutoDiffXd& mass = test_particle.mass;
  mass.value() = 20.0;
  mass.derivatives().resize(num_variables_differentiate_with_respect_to);
  mass.derivatives()(index_for_partial_time) = 0.0;
  mass.derivatives()(index_for_partial_x) = 0.0;

  // The state vector has the following assignments:
  // state[0] = x and state[1] = xDt.
  // Set the value of the variables (state[0] and state[1]) to the value at
  // which the derivative is to be evaluated.  Arbitrarily chose [0.5, 0.7].
  AutoDiffXd state[2];
  state[0].value() = 0.5;
  state[1].value() = 0.7;

  // Provide enough room for differentiation with respect to three variables:
  // partial of x, xDt, and t with respect to t.
  state[0].derivatives().resize(num_variables_differentiate_with_respect_to);
  state[1].derivatives().resize(num_variables_differentiate_with_respect_to);

  // Set the derivative of state[0] with respect to state[0] (itself) to 1.
  // Set the derivative of state[0] with respect to state[1] to 0.
  state[0].derivatives()(index_for_partial_time) = 0.0;
  state[0].derivatives()(index_for_partial_x) = 1.0;

  // Set the derivative of state[1] with respect to state[0] to 0.
  // Set the derivative of state[1] with respect to state[1] (itself) to 1.
  state[1].derivatives()(index_for_partial_time) = 0.0;
  state[1].derivatives()(index_for_partial_x) = 0.0;

  // stateDt is calculated, hence its value and derivatives will be assigned.
  AutoDiffXd stateDt[2];
  test_particle.CalcDerivativesToStateDt(test_time, state, stateDt);

  // Value: m = mass (assigned above).
  // ∂m/∂t = 0.
  // ∂m/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.mass, mass.value(), 0,
                                           0);

  // Value: x = state[0] (assigned above).
  // ∂x/∂t = 0.
  // ∂x/∂x = 1.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.x, state[0].value(), 0,
                                           1);

  // Value: ẋ = state[1] (assigned above).
  // ∂ẋ/∂t = 0.
  // ∂ẋ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(test_particle.xDt, state[1].value(),
                                           0, 0);

  // Value: ẍ = cos(t)/m.
  // ∂ẍ/∂t = -sint(t)/m.
  // ∂ẍ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(
      test_particle.xDDt, cos(test_time.value()) / mass.value(),
      -sin(test_time.value()) / mass.value(), 0);

  // Value: f = cos(t).
  // ∂f/∂t = -sin(t).
  // ∂f/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(
      test_particle.F, cos(test_time.value()), -sin(test_time.value()), 0);
}

} // namespace
} // namespace particle1d
} // namespace examples
} // namespace drake

#include "drake/examples/particle1d/particle1d_manual.h"

#include <gtest/gtest.h>

#include <vector>
#include "drake/math/autodiff_gradient.h"

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
// Note: partial with respect to variable A (∂y/∂A) will be denoted as dy_dA.
void VerifyAutoDiffValueAndPartialDerivatives(
    const AutoDiffXd& y, double y_value,
    double dy_dA, double dy_dB, double dy_dC) {
  // First test the value of the AutoDiff evaluated at the value set in the
  // AutoDiff declaration.
  VerifyValueOnly(y.value(), y_value);

  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

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
GTEST_TEST(PlantTest, ManualClassTestAutoDiffDerivativeTest) {
  // Note: This test sets up the AutoDiff values and derivative vectors
  // explicitly as a way to pedagogically show what Autodiff is doing, and how
  // to take the partial derivatives. The explicit declaration of values and
  // derivatives will not scale in this fashion to anything larger than this
  // example (three partials) due to the large amount of coding overhead. A more
  // scalable method would be to implement an AutoDiff using
  // initializeAutoDiffGivenGradientMatrix() as shown in the following commented
  // section (tests given to verify initialization and partial differentiation).
  // ---------------------------------------------------------------------------
  //  Eigen::Vector3d partial_variables;
  //  partial_variables[0] = 20;  // Variable mass
  //  partial_variables[1] = 2;   // Variable time
  //  partial_variables[2] = 0.5; // Variable x
  //  Eigen::Matrix<double, 3, 3> gradient;
  //  gradient.setIdentity();
  //
  //  auto outauto =
  //    drake::math::initializeAutoDiffGivenGradientMatrix(ad_states, gradient);
  //  EXPECT_EQ(outauto.rows(), 3);
  //  EXPECT_EQ(outauto[0].derivatives().size(), 3);
  //  EXPECT_EQ(outauto[0].derivatives()(0), 1);
  //  EXPECT_EQ(outauto[0].derivatives()(1), 0);
  //  EXPECT_EQ(outauto[0].derivatives()(2), 0);
  // ---------------------------------------------------------------------------

  Particle1dManual<AutoDiffXd> test_particle;

  Particle1dManual<AutoDiffXd>::ParticleData& particle_data =
                                              test_particle.get_particle_data();

  // Calculate partial derivatives with respect to 2 variables (time and x).
  const int num_variables_differentiate_with_respect_to = 3;
  const int index_for_partial_mass = 0;
  const int index_for_partial_time = 1;
  const int index_for_partial_x = 2;

  // Set the value of the particle's mass. Arbitrarily chosen to be 20 kg.
  AutoDiffXd& mass = particle_data.mass_;
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
  state[0].derivatives()(index_for_partial_mass) = 0.0;
  state[0].derivatives()(index_for_partial_time) = 0.0;
  state[0].derivatives()(index_for_partial_x) = 1.0;

  // stateDt is calculated, hence its value and derivatives will be assigned.
  AutoDiffXd stateDt[2];
  test_particle.CalcDerivativesToStateDt(time, state, stateDt);

  // Value: m = mass (assigned above).
  // The expected values are: ∂m/∂m = 1, ∂m/∂t = 0, ∂m/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.mass_, mass.value(), 1,
                                           0, 0);

  // Value: x = state[0] (assigned above).
  // The expected values are: ∂x/∂m = 0, ∂x/∂t = 0, ∂x/∂x = 1.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.x_, state[0].value(),
                                           0, 0, 1);

  // Value: ẋ = state[1] (assigned above).
  // The expected values are: ∂ẋ/∂m = 0, ∂ẋ/∂t = 0, ∂ẋ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(stateDt[0], state[1].value(), 0, 0,
                                           0);

  // Value: ẍ = cos(t)/m.
  // The expected values are: ∂ẍ/∂m = -cos(t)/m², ∂ẍ/∂t = -sint(t)/m, ∂ẍ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(
      stateDt[1], cos(time.value()) / mass.value(),
      -cos(time.value()) / (mass.value() * mass.value()),
      -sin(time.value()) / mass.value(), 0);

  // Value: f = cos(t).
  // The expected values are: ∂f/∂m = 0, ∂f/∂t = -sin(t), ∂f/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.F_, cos(time.value()),
                                           0, -sin(time.value()), 0);
}

// Unit test for the Particle1dManual class with AutoDiffXd instantiation.
// Note: This unit test is more complicated than the previous unit test as it
// checks partial derivative calculations (whereas the previous test only checks
// AutoDiff values).
GTEST_TEST(PlantTest, ShortAutoDiff) {
  // Note: This test sets up the AutoDiff values and derivative vectors
  // explicitly as a way to pedagogically show what Autodiff is doing, and how
  // to take the partial derivatives. The explicit declaration of values and
  // derivatives will not scale in this fashion to anything larger than this
  // example (three partials) due to the large amount of coding overhead. A more
  // scalable method would be to implement an AutoDiff using
  // initializeAutoDiffGivenGradientMatrix() as shown in the following commented
  // section (tests given to verify initialization and partial differentiation).
  // ---------------------------------------------------------------------------
  //  Eigen::Vector3d partial_variables;
  //  partial_variables[0] = 20;  // Variable mass
  //  partial_variables[1] = 2;   // Variable time
  //  partial_variables[2] = 0.5; // Variable x
  //  Eigen::Matrix<double, 3, 3> gradient;
  //  gradient.setIdentity();
  //
  //  auto outauto =
  //    drake::math::initializeAutoDiffGivenGradientMatrix(ad_states, gradient);
  //  EXPECT_EQ(outauto.rows(), 3);
  //  EXPECT_EQ(outauto[0].derivatives().size(), 3);
  //  EXPECT_EQ(outauto[0].derivatives()(0), 1);
  //  EXPECT_EQ(outauto[0].derivatives()(1), 0);
  //  EXPECT_EQ(outauto[0].derivatives()(2), 0);
  // ---------------------------------------------------------------------------

  Particle1dManual<AutoDiffXd> test_particle;

  Particle1dManual<AutoDiffXd>::ParticleData& particle_data =
      test_particle.get_particle_data();

  AutoDiffXd state[2] = {0.5, 0.7};
  AutoDiffXd stateDt[2];

  // Set up the AutoDiff scalar with the gradient matrix.
  Eigen::Vector3d partial_variables;
  partial_variables[0] = 3; // Variable mass
  partial_variables[1] = 2;  // Variable time
  partial_variables[2] = state[0].value(); // Variable x
  Eigen::Matrix<double, 3, 3> gradient_matrix;
  gradient_matrix.setIdentity();
/*
  auto outauto =
    drake::math::initializeAutoDiffGivenGradientMatrix(ad_states, gradient_matrix);
  EXPECT_EQ(outauto.rows(), 4);
  EXPECT_EQ(outauto[0].derivatives().size(), 4);
  EXPECT_EQ(outauto[0].derivatives()(0), 1);
  EXPECT_EQ(outauto[0].derivatives()(1), 0);
  EXPECT_EQ(outauto[0].derivatives()(2), 0);
  EXPECT_EQ(outauto[0].derivatives()(3), 0);

  test_particle.CalcDerivativesToStateDt(time, state, stateDt);

  // Value: m = mass (assigned above).
  // The expected values are: ∂m/∂m = 1, ∂m/∂t = 0, ∂m/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.mass_, mass.value(), 1,
                                           0, 0);

  // Value: x = state[0] (assigned above).
  // The expected values are: ∂x/∂m = 0, ∂x/∂t = 0, ∂x/∂x = 1.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.x_, state[0].value(),
                                           0, 0, 1);

  // Value: ẋ = state[1] (assigned above).
  // The expected values are: ∂ẋ/∂m = 0, ∂ẋ/∂t = 0, ∂ẋ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(stateDt[0], state[1].value(), 0, 0,
                                           0);

  // Value: ẍ = cos(t)/m.
  // The expected values are: ∂ẍ/∂m = -cos(t)/m², ∂ẍ/∂t = -sint(t)/m, ∂ẍ/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(
      stateDt[1], cos(time.value()) / mass.value(),
      -cos(time.value()) / (mass.value() * mass.value()),
      -sin(time.value()) / mass.value(), 0);

  // Value: f = cos(t).
  // The expected values are: ∂f/∂m = 0, ∂f/∂t = -sin(t), ∂f/∂x = 0.
  VerifyAutoDiffValueAndPartialDerivatives(particle_data.F_, cos(time.value()),
                                           0, -sin(time.value()), 0);*/
}

}  // namespace
}  // namespace particle1d
}  // namespace examples
}  // namespace drake

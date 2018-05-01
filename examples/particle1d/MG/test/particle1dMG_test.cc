#include "drake/examples/particle1d/MG/particle1dMG.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace examples {
namespace particle1d {
namespace {

// Unit test for the Particle1dMG class with Double instantiation.
GTEST_TEST(PlantTest, MGClassTestDouble) {
  MotionGenesis::Particle1dMG<double> test_particle;
  double time = 0.0;
  double state[2] = {0};
  double stateDt[2] = {0};
  double test_mass = 1;

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(test_particle.mass == 1);

  // for loop to check the repeated statements
  for (int i = 0; i < 3; i++) {
    if (i == 1) {
      // Test calculations for another value of time and another state.
      time = 0.75;
      state[0] = 0.26831113112;
      state[1] = 0.68163876002;
    }else if(i== 2){
      // Test calculations for another value of time and another state.
      time = 0.9;
      test_mass = 25;
      test_particle.mass = test_mass; // mass to 25 kg
      state[0] = 0.37839003172;
      state[1] = 0.78332690962;
    }
    // Ensure time-derivative of state is properly calculated when state has
    // initial values of zero.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    test_particle.CalcDerivativesToStateDt(time, state, stateDt);
    EXPECT_EQ(stateDt[0], state[1]);                     // ẋ = ẋ
    EXPECT_EQ(stateDt[1], cos(time)/test_particle.mass); // ẍ = cos(time)/m

    // Test if mass is properly assigned and did not change during the
    // calculation step.
    EXPECT_TRUE(test_particle.mass == test_mass);
  }
}

// Unit test for the Particle1dMG class with AutoDiffXd instantiation.
GTEST_TEST(PlantTest, MGClassTestAutoDiffValuesOnly) {
  MotionGenesis::Particle1dMG<AutoDiffXd> test_particle;
  AutoDiffXd time = 0.0;
  AutoDiffXd state[2] = {0};
  AutoDiffXd stateDt[2] = {0};
  AutoDiffXd test_mass = 1;

  // Ensure the default initialization of mass is 1 kg.
  EXPECT_TRUE(test_particle.mass == 1);

  // for loop to check the repeated statements
  for (int i = 0; i < 3; i++) {
    if (i == 1) {
      // Test calculations for another value of time and another state.
      time = 0.75;
      state[0] = 0.26831113112;
      state[1] = 0.68163876002;
    } else if (i == 2) {
      // Test calculations for another value of time and another state.
      time = 0.9;
      test_mass = 25;
      test_particle.mass = test_mass;  // mass to 25 kg
      state[0] = 0.37839003172;
      state[1] = 0.78332690962;
    }
    // Ensure time-derivative of state is properly calculated when state has
    // initial values of zero.
    // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
    test_particle.CalcDerivativesToStateDt(time, state, stateDt);
    EXPECT_EQ(stateDt[0], state[1]);                        // ẋ = ẋ
    EXPECT_EQ(stateDt[1], cos(time) / test_particle.mass);  // ẍ = cos(time)/m

    // Test if mass is properly assigned and did not change during the
    // calculation step.
    EXPECT_EQ(test_particle.mass, test_mass);
 }
}

} // namespace
} // namespace particle1d
} // namespace examples
} // namespace drake

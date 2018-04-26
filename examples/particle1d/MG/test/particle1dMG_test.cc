#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

#include "drake/examples/particle1d/MG/particle1dMG.h"

namespace drake {
namespace examples {
namespace particle1d {
namespace {

GTEST_TEST(PlantTest, MGClassTest) {
  // unit test for the Particle1dMG class
  MotionGenesis::Particle1dMG<double> test_particle;
  double time = 0.0;
  double state[2] = {0};
  double stateDt[2] = {0};

  // Ensure the default initializition of mass is 1 kg.
  EXPECT_TRUE(test_particle.m == 1);

  // Ensure time-derivative of state is properly calculated at time = 0.0.
  // Note:  From f = mẍ ,  ẍ  = f/m  = cos(time)/m.
  test_particle.CalcDerivativesToStateDt(time, state, stateDt);
  EXPECT_EQ(stateDt[0], state[1]);                  // ẋ = ẋ
  EXPECT_EQ(stateDt[1], cos(time)/test_particle.m); // ẍ = cos(time)/m

  // Test calculations for another value of time.
  time = 0.75;
  state[0] = 0.26831113112;
  state[1] = 0.68163876002;

  // Ensure the mass of the particle has not changed.
  EXPECT_TRUE(test_particle.m == 1);

  // Ensure time-derivative of state is properly calculated at time = 0.75
  test_particle.CalcDerivativesToStateDt(time, state, stateDt);
  EXPECT_EQ(stateDt[0], state[1]);                  // ẋ = ẋ
  EXPECT_EQ(stateDt[1], cos(time)/test_particle.m); // ẍ = cos(time)/m

  // Test calculations for another value of time and another value of mass.
  time = 0.9;
  test_particle.m = 25; // mass to 25 kg
  state[0] = 0.37839003172;
  state[1] = 0.78332690962;

  // Ensure time-derivative of state is properly calculated at time = 0.9.
  test_particle.CalcDerivativesToStateDt(time, state, stateDt);
  EXPECT_EQ(stateDt[0], state[1]);                  // ẋ = ẋ
  EXPECT_EQ(stateDt[1], cos(time)/test_particle.m); // ẍ = cos(time)/m
  EXPECT_TRUE(test_particle.m == 25);
}

} // namespace
} // namespace particle1d
} // namespace examples
} // namespace drake

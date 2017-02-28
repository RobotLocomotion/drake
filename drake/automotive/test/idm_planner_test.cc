#include "drake/automotive/idm_planner.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

// Set the initial states such that the agent and ego start at the
// headway distance, both at the desired speed.
GTEST_TEST(IdmPlannerTest, SameSpeedAtHeadwayDistance) {
  IdmPlannerParameters<double>* params = nullptr;
  IdmPlanner<double>::SetDefaultParameters(params);
  const double result =
      IdmPlanner<double>::Evaluate(
          *params, params->v_ref() /* set ego velocity to v_ref */,
          params->v_ref() * params->time_headway() /* maintain time headway */,
          0. /* ego and lead car at same speed */ );
  // Expect there to be no acceleration or deceleration.
  EXPECT_NEAR(result, 0., 1e-2);
}

/*
// Set the initial states such that the agent and ego start within the
// headway distance, both at the desired speed.
GTEST_TEST(IdmPlannerTest, SameSpeedBelowHeadwayDistance) {
  std::vector<double> state = {0.0, IdmPlannerTest::get_v_0(), 6.0,
                               IdmPlannerTest::get_v_0()};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->CalcOutput(*context_, output_.get());
  // Expect the car to decelerate.
  EXPECT_LE(result->GetAtIndex(0), -1e-2);
}

// Set the initial states such that the agent and ego start close
// together at different speeds.
GTEST_TEST(IdmPlannerTest, DifferentSpeedsBelowHeadwayDistance) {
  std::vector<double> state = {0.0, 7.0, 6.0, 4.0};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->CalcOutput(*context_, output_.get());
  // Expect the car to decelerate.
  EXPECT_LE(result->GetAtIndex(0), -1e-2);
}

// Set the agent and ego sufficiently far apart from one another, with
// the ego car initially at the desired speed.  set-point.
GTEST_TEST(IdmPlannerTest, EgoAtDesiredSpeed) {
  std::vector<double> state = {0.0, IdmPlannerTest::get_v_0(), 1e6, 0.0};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->CalcOutput(*context_, output_.get());
  // Expect there to be no acceleration or deceleration.
  EXPECT_NEAR(result->GetAtIndex(0), 0.0, 1e-2);
}

// Set the agent and ego sufficiently far apart from one another, with
// the ego car speed initially zero.  set-point.
GTEST_TEST(IdmPlannerTest, EgoStartFromRest) {
  std::vector<double> state = {0.0, 0.0, 1e6, 0.0};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->CalcOutput(*context_, output_.get());
  // Expect the car to accelerate.
  EXPECT_GE(result->GetAtIndex(0), 1e-2);
}
*/
}  // namespace
}  // namespace automotive
}  // namespace drake

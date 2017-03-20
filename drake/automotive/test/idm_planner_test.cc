#include "drake/automotive/idm_planner.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

using std::sqrt;

class IdmPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set the parameters to their default values.
    IdmPlanner<double>::SetDefaultParameters(params_.get());
  }
  std::unique_ptr<IdmPlannerParameters<double>> params_ =
      std::make_unique<IdmPlannerParameters<double>>();
};

// Set the initial states such that the agent and ego start at the
// headway distance, with the ego car closing in on the lead car.
TEST_F(IdmPlannerTest, SameSpeedAtHeadwayDistance) {
  const double ego_velocity = params_->v_ref();
  const double target_distance = params_->v_ref() * params_->time_headway();
  const double target_distance_dot =
      -4 * sqrt(params_->a() * params_->b()) / params_->v_ref();

  const double result = IdmPlanner<double>::Evaluate(
      *params_, ego_velocity, target_distance, target_distance_dot);

  // We expect acceleration to be close to zero.
  EXPECT_NEAR(0., result, 1e-2);
}

// Set the initial states such that the agent and ego start within the
// headway distance, both at the desired speed.
TEST_F(IdmPlannerTest, SameSpeedBelowHeadwayDistance) {
  const double ego_velocity = params_->v_ref();
  const double target_distance = 6.;
  const double target_distance_dot = 0.;

  const double result = IdmPlanner<double>::Evaluate(
      *params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to decelerate.
  EXPECT_GE(0., result);
}

// Set the initial states such that the agent and ego start close
// together at different speeds.
TEST_F(IdmPlannerTest, DifferentSpeedsBelowHeadwayDistance) {
  const double ego_velocity = 7.;
  const double target_distance = 6.;
  const double target_distance_dot = 3.;

  const double result = IdmPlanner<double>::Evaluate(
      *params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to decelerate.
  EXPECT_GE(0., result);
}

// Set the agent and ego sufficiently far apart from one another, with
// the ego car initially at the desired speed.
TEST_F(IdmPlannerTest, EgoAtDesiredSpeed) {
  const double ego_velocity = params_->v_ref();
  const double target_distance = 1e6;
  const double target_distance_dot = params_->v_ref();

  const double result = IdmPlanner<double>::Evaluate(
      *params_, ego_velocity, target_distance, target_distance_dot);

  // We expect acceleration to be close to zero.
  EXPECT_NEAR(0., result, 1e-2);
}

// Set the agent and ego sufficiently far apart from one another, with
// the ego car speed initially zero.
TEST_F(IdmPlannerTest, EgoStartFromRest) {
  const double ego_velocity = 0.;
  const double target_distance = 1e6;
  const double target_distance_dot = 0.;

  const double result = IdmPlanner<double>::Evaluate(
      *params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to accelerate.
  EXPECT_LE(0., result);
}

}  // namespace
}  // namespace automotive
}  // namespace drake

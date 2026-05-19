#include "drake/planning/experimental/mbp_environment_collision_checker.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/planning/experimental/test/sphere_robot_model_collision_checker_abstract_test_suite.h"
#include "drake/planning/test/planning_test_helpers.h"
#include "drake/planning/test_utilities/collision_checker_abstract_test_suite.h"

using drake::planning::test::CollisionCheckerAbstractTestSuite;
using drake::planning::test::CollisionCheckerConstructionParams;
using drake::planning::test::CollisionCheckerTestParams;
using drake::planning::test::MakeCollisionCheckerTestScene;
using drake::planning::test::MakePlanningTestModel;
using drake::planning::test::MakeWeightedIiwaConfigurationDistanceFunction;

namespace drake {
namespace planning {
namespace experimental {
namespace test {
namespace {

CollisionCheckerTestParams MakeMbpEnvironmentCollisionCheckerParams() {
  CollisionCheckerTestParams result;
  const CollisionCheckerConstructionParams p;
  auto model = MakePlanningTestModel(MakeCollisionCheckerTestScene());
  const auto robot_instance = model->plant().GetModelInstanceByName("iiwa");
  result.checker.reset(new MbpEnvironmentCollisionChecker(
      {.model = std::move(model),
       .robot_model_instances = {robot_instance},
       .configuration_distance_function =
           MakeWeightedIiwaConfigurationDistanceFunction(),
       .edge_step_size = p.edge_step_size,
       .env_collision_padding = p.env_padding,
       .self_collision_padding = p.self_padding}));
  return result;
}

}  // namespace

INSTANTIATE_TEST_SUITE_P(
    MbpEnvironmentCollisionCheckerTest, CollisionCheckerAbstractTestSuite,
    testing::Values(MakeMbpEnvironmentCollisionCheckerParams()));

INSTANTIATE_TEST_SUITE_P(
    MbpEnvironmentCollisionCheckerTest2,
    SphereRobotModelCollisionCheckerAbstractTestSuite,
    testing::Values(SphereRobotModelCollisionCheckerTestParams{
        MakeMbpEnvironmentCollisionCheckerParams().checker}));

}  // namespace test
}  // namespace experimental
}  // namespace planning
}  // namespace drake

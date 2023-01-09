#include "planning/unimplemented_collision_checker.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "planning/robot_diagram_builder.h"

namespace anzu {
namespace planning {
namespace {

using drake::multibody::default_model_instance;
using drake::planning::CollisionCheckerParams;

class MyUnimplementedCollisionChecker : public UnimplementedCollisionChecker {
 public:
  using UnimplementedCollisionChecker::UnimplementedCollisionChecker;
};

GTEST_TEST(UnimplementedCollisionCheckerTest, SmokeTest) {
  CollisionCheckerParams params{
    .model = RobotDiagramBuilder<double>{}.BuildDiagram(),
    .robot_model_instances = {default_model_instance()},
    .configuration_distance_function = [](auto...) { return 0; },
    .edge_step_size = 0.1,
  };

  // Prove that the class is concrete (no missing overrides).
  const MyUnimplementedCollisionChecker dut(std::move(params), false);

  // Sanity check one error message string.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Clone(), "DoClone is not implemented");
}

}  // namespace
}  // namespace planning
}  // namespace anzu

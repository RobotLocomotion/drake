#include "drake/planning/unimplemented_collision_checker.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace {

using drake::multibody::default_model_instance;

class MyUnimplementedCollisionChecker : public UnimplementedCollisionChecker {
 public:
  MyUnimplementedCollisionChecker(CollisionCheckerParams params,
                                  bool supports_parallel_checking)
      : UnimplementedCollisionChecker(std::move(params),
                                      supports_parallel_checking) {
    // We need to allocate contexts so that the DoClone() test below fails
    // on UnimplementedCollisionChecker's DoClone() and not Clone().
    AllocateContexts();
  }
};

GTEST_TEST(UnimplementedCollisionCheckerTest, SmokeTest) {
  CollisionCheckerParams params{
      .model = RobotDiagramBuilder<double>{}.Build(),
      .robot_model_instances = {default_model_instance()},
      .configuration_distance_function =
          [](auto...) {
            return 0;
          },
      .edge_step_size = 0.1,
  };

  // Prove that the class is concrete (no missing overrides).
  const MyUnimplementedCollisionChecker dut(std::move(params), false);

  // Sanity check one error message string.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Clone(), "DoClone is not implemented");
}

}  // namespace
}  // namespace planning
}  // namespace drake

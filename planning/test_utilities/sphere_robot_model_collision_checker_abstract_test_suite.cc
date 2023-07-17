#include "planning/test/sphere_robot_model_collision_checker_abstract_test_suite.h"

#include <unordered_map>
#include <vector>

namespace anzu {
namespace planning {
namespace test {

using drake::geometry::GeometryId;
using drake::multibody::BodyIndex;

TEST_P(SphereRobotModelCollisionCheckerAbstractTestSuite, SphereChecker) {
  std::shared_ptr<drake::planning::CollisionChecker> checker = GetParam();
  auto& sphere_checker =
      dynamic_cast<SphereRobotModelCollisionChecker&>(*checker);

  // Test adding an extension to the robot that causes collision
  const BodyIndex test_body_index(8);
  const std::unordered_map<GeometryId, SphereSpecification>&
      raw_reference_spheres =
      sphere_checker.RobotCollisionModel().at(test_body_index);
  std::vector<SphereSpecification> reference_spheres;
  for (const auto& [sphere_id, sphere] : raw_reference_spheres) {
    drake::unused(sphere_id);
    reference_spheres.emplace_back(sphere);
  }
  std::vector<SphereSpecification> test_spheres;
  test_spheres.push_back(SphereSpecification(0.0, 0.0, 0.0, 1.0));
  sphere_checker.UpdateBodyCollisionModel(
      test_body_index, test_spheres, true);
  const Eigen::VectorXd q0{Eigen::VectorXd::Zero(7)};
  EXPECT_FALSE(sphere_checker.CheckConfigCollisionFree(q0));
  // Test resetting the link model
  sphere_checker.UpdateBodyCollisionModel(
      test_body_index, reference_spheres, false);
  EXPECT_TRUE(sphere_checker.CheckConfigCollisionFree(q0));
}

}  // namespace test
}  // namespace planning
}  // namespace anzu

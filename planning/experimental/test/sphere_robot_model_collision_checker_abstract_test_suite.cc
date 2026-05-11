#include "drake/planning/dev/test/sphere_robot_model_collision_checker_abstract_test_suite.h"

#include <unordered_map>
#include <vector>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace planning {
namespace test {

std::ostream& operator<<(std::ostream& out,
                         const SphereRobotModelCollisionCheckerTestParams& p) {
  out << drake::NiceTypeName::Get(*p.checker);
  return out;
}

TEST_P(SphereRobotModelCollisionCheckerAbstractTestSuite, SphereChecker) {
  std::shared_ptr<CollisionChecker> checker = GetParam().checker;
  auto& sphere_checker =
      dynamic_cast<SphereRobotModelCollisionChecker&>(*checker);

  // Test adding an extension to the robot that causes collision
  const multibody::BodyIndex test_body_index(8);
  const BodySpheres& raw_reference_spheres =
      sphere_checker.RobotCollisionModel().at(test_body_index);
  std::vector<SphereSpecification> reference_spheres;
  for (const auto& [sphere_id, sphere] : raw_reference_spheres) {
    drake::unused(sphere_id);
    reference_spheres.emplace_back(sphere);
  }
  std::vector<SphereSpecification> test_spheres;
  test_spheres.push_back(SphereSpecification(0.0, 0.0, 0.0, 1.0));
  sphere_checker.UpdateBodyCollisionModel(test_body_index, test_spheres, true);
  const Eigen::VectorXd q0{Eigen::VectorXd::Zero(7)};
  EXPECT_FALSE(sphere_checker.CheckConfigCollisionFree(q0));
  // Test resetting the link model
  sphere_checker.UpdateBodyCollisionModel(test_body_index, reference_spheres,
                                          false);
  EXPECT_TRUE(sphere_checker.CheckConfigCollisionFree(q0));
}

}  // namespace test
}  // namespace planning
}  // namespace drake

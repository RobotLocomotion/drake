#include "drake/planning/experimental/mbp_environment_collision_checker.h"

#include <stdexcept>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/planning/experimental/test/sphere_robot_model_collision_checker_abstract_test_suite.h"
#include "drake/planning/robot_diagram_builder.h"
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

// A planar arm with three revolute joints about z. Links 1 and 3 each carry a
// single sphere; link 2 carries none.
constexpr char kPlanarArmUrdf[] = R"""(
<robot name="arm">
  <link name="link1">
    <inertial><mass value="1"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision><origin xyz="0.5 0 0"/>
      <geometry><sphere radius="0.1"/></geometry>
    </collision>
  </link>
  <link name="link2">
    <inertial><mass value="1"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="link3">
    <inertial><mass value="1"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision><origin xyz="0.5 0 0"/>
      <geometry><sphere radius="0.1"/></geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/><child link="link1"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="1 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/><origin xyz="1 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
)""";

// A sphere obstacle welded to the world.
constexpr char kObstacleUrdf[] = R"""(
<robot name="obstacle">
  <link name="obstacle">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
  <joint name="weld" type="fixed">
    <parent link="world"/><child link="obstacle"/>
    <origin xyz="0.028 -0.233 0"/>
  </joint>
</robot>
)""";

// Checks the clearance Jacobians against finite differences of the clearance
// distances. The arm is folded so that link 3's sphere is within the influence
// distance of both link 1's sphere and the obstacle, while link 1's sphere is
// out of the obstacle's range. Each row therefore comes from a single pair of
// spheres, so its Jacobian is exactly the derivative of its distance.
GTEST_TEST(MbpEnvironmentCollisionCheckerClearanceTest,
           JacobiansMatchDistances) {
  RobotDiagramBuilder<double> builder;
  const multibody::ModelInstanceIndex arm =
      builder.parser().AddModelsFromString(kPlanarArmUrdf, "urdf").at(0);
  builder.parser().AddModelsFromString(kObstacleUrdf, "urdf");
  const MbpEnvironmentCollisionChecker checker(
      {.model = builder.Build(),
       .robot_model_instances = {arm},
       .configuration_distance_function =
           [](const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
             return (q1 - q2).norm();
           },
       .edge_step_size = 0.05});
  const multibody::BodyIndex link1 =
      checker.plant().GetBodyByName("link1").index();
  const multibody::BodyIndex link3 =
      checker.plant().GetBodyByName("link3").index();
  const multibody::BodyIndex obstacle =
      checker.plant().GetBodyByName("obstacle").index();

  const double kInfluenceDistance = 0.3;
  const Eigen::Vector3d q(0.3, 2.6, 2.15);
  const RobotClearance clearance =
      checker.CalcRobotClearance(q, kInfluenceDistance);

  // Returns the distance of the row of `perturbed` that matches `row` of
  // `clearance`.
  auto matching_distance = [&](const RobotClearance& perturbed, int row) {
    for (int i = 0; i < perturbed.size(); ++i) {
      if (perturbed.robot_indices()[i] == clearance.robot_indices()[row] &&
          perturbed.other_indices()[i] == clearance.other_indices()[row] &&
          perturbed.collision_types()[i] == clearance.collision_types()[row]) {
        return perturbed.distances()(i);
      }
    }
    throw std::logic_error("No matching row in the perturbed clearance.");
  };

  int num_environment_rows = 0;
  int num_self_rows = 0;
  for (int row = 0; row < clearance.size(); ++row) {
    const multibody::BodyIndex robot_body = clearance.robot_indices()[row];
    const multibody::BodyIndex other_body = clearance.other_indices()[row];
    if (clearance.collision_types()[row] ==
        RobotCollisionType::kEnvironmentCollision) {
      ++num_environment_rows;
      EXPECT_EQ(robot_body, link3);
      EXPECT_EQ(other_body, obstacle);
    } else {
      ++num_self_rows;
      EXPECT_EQ(other_body, robot_body == link1 ? link3 : link1);
      // Joint 1 moves links 1 and 3 together, so it cannot change the
      // distance between them.
      EXPECT_NEAR(clearance.jacobians()(row, 0), 0.0, 1e-12);
    }

    const double kDelta = 1e-6;
    Eigen::RowVector3d jacobian_finite_difference;
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d q_plus = q;
      Eigen::Vector3d q_minus = q;
      q_plus(j) += kDelta;
      q_minus(j) -= kDelta;
      jacobian_finite_difference(j) =
          (matching_distance(
               checker.CalcRobotClearance(q_plus, kInfluenceDistance), row) -
           matching_distance(
               checker.CalcRobotClearance(q_minus, kInfluenceDistance), row)) /
          (2 * kDelta);
    }
    EXPECT_TRUE(CompareMatrices(clearance.jacobians().row(row),
                                jacobian_finite_difference, 1e-8));
  }
  // Link 3's sphere has both an environment row and a self row, and link 1's
  // sphere has a self row only.
  EXPECT_EQ(num_environment_rows, 1);
  EXPECT_EQ(num_self_rows, 2);
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

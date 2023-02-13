#include "drake/planning/scene_graph_collision_checker.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/test/planning_test_helpers.h"
#include "drake/planning/test_utilities/collision_checker_abstract_test_suite.h"

namespace drake {
namespace planning {
namespace test {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::BodyIndex;
using testing::ElementsAre;

CollisionCheckerTestParams MakeSceneGraphCollisionCheckerParams() {
  CollisionCheckerTestParams result;
  const CollisionCheckerConstructionParams p;
  auto model = MakePlanningTestModel(MakeCollisionCheckerTestScene());
  const auto robot_instance = model->plant().GetModelInstanceByName("iiwa");
  result.checker.reset(new SceneGraphCollisionChecker({
      .model = std::move(model),
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
    SceneGraphCollisionCheckerTestSuite, CollisionCheckerAbstractTestSuite,
    testing::Values(MakeSceneGraphCollisionCheckerParams()));

// Creates three spheres (each on a prismatic joint with its parent set to
// the world origin) and checks their RobotClearance query.
//
// The three spheres are configured so that when q = (5, 5, 0) (for Boston,
// Seattle, and Dallas, respectively), the distances from Do to So and Bo are 5
// (they form 3-4-5 triangles) and the distance between So and Bo is 8.
//
//       So                      Bo
//       |  ╲                  ╱  |
//       |    ╲  5        5  ╱    |
//    3  |      ╲          ╱      |  3
//       |        ╲      ╱        |
//       |          ╲  ╱          |
//       └-----------Do-----------┘
//            4             4
//
// Boston's sphere has a radius of 0.5.
// Seattle's sphere has a radius of 0.25.
// Dallas's sphere has a radius of 1.0.
GTEST_TEST(SceneGraphCollisionCheckerTest, ClearanceThreeSpheres) {
  RobotDiagramBuilder<double> builder;
  builder.mutable_parser().AddModelsFromString(R"""(
<?xml version='1.0'?>
<sdf xmlns:drake='http://drake.mit.edu' version='1.9'>
<world name='default'>
  <model name='robot'>
    <link name='boston'>
      <collision name='boston_collision'>
        <geometry><sphere><radius>0.5</radius></sphere></geometry>
      </collision>
    </link>
    <joint name='boston_joint' type='prismatic'>
      <parent>world</parent>
      <child>boston</child>
      <axis><xyz>0.8 0.6 0</xyz></axis>
    </joint>
    <link name='seattle'>
      <collision name='seattle_collision'>
        <geometry><sphere><radius>0.25</radius></sphere></geometry>
      </collision>
    </link>
    <joint name='seattle_joint' type='prismatic'>
      <parent>world</parent>
      <child>seattle</child>
      <axis><xyz>-0.8 0.6 0</xyz></axis>
    </joint>
  </model>
  <model name='environment'>
    <link name='dallas'>
      <collision name='dallas_collision'>
        <geometry><sphere><radius>1.0</radius></sphere></geometry>
      </collision>
    </link>
    <joint name='dallas_joint' type='prismatic'>
      <parent>world</parent>
      <child>dallas</child>
      <axis><xyz>0 -1 0</xyz></axis>
    </joint>
  </model>
</world>
</sdf>
)""", "sdf");
  const auto& plant = builder.plant();

  CollisionCheckerParams params;
  params.model = builder.Build();
  params.robot_model_instances.push_back(
      plant.GetModelInstanceByName("robot"));
  params.configuration_distance_function =
      [](const VectorXd& q1, const VectorXd& q2) {
    return (q1 - q2).norm();
  };
  params.edge_step_size = 0.05;
  SceneGraphCollisionChecker dut(std::move(params));

  const BodyIndex boston = plant.GetBodyByName("boston").index();
  const BodyIndex seattle = plant.GetBodyByName("seattle").index();
  const BodyIndex dallas = plant.GetBodyByName("dallas").index();

  // Posture Seattle's center at (-4, 3), Boston's center at (4, 3), and
  // Dallas's center at (0, 0); see the test overview above for a figure.
  Vector3d q(5.0, 5.0, 0.0);
  const double influence = 99999999;
  auto clearance = dut.CalcRobotClearance(q, influence);

  // The current order of return values is like so:
  //
  //  boston, seattle
  //  boston, dallas
  //  seattle, dallas
  //
  // We don't care what the order is, but it's easier for now to just hard-code
  // one in the test.
  const double tol = 1e-9;
  EXPECT_EQ(clearance.size(), 3);
  EXPECT_EQ(clearance.num_positions(), 3);
  EXPECT_THAT(clearance.robot_indices(), ElementsAre(
      boston,
      boston,
      seattle));
  EXPECT_THAT(clearance.other_indices(), ElementsAre(
      seattle,
      dallas,
      dallas));
  EXPECT_THAT(clearance.collision_types(),
              ElementsAre(RobotCollisionType::kSelfCollision,
                          RobotCollisionType::kEnvironmentCollision,
                          RobotCollisionType::kEnvironmentCollision));
  // The Boston <-> Seattle distance on-center is 8.0.
  // The Boston <-> Dallas distance on-center is 5.0.
  // The Seattle <-> Dallas distance on-center is 5.0.
  // In each case, the measured distance is less than on-center by each of the
  // two cities' sphere radii.
  EXPECT_TRUE(CompareMatrices(clearance.distances(), (VectorXd(3) <<
      8.0 - 0.5 - 0.25,
      5.0 - 0.5 - 1.0,
      5.0 - 0.25 - 1.0).finished(), tol));
  // Recall that:
  // - The prismatic joint for Boston moves (+0.6, +0.8) for each +1.0 q₀.
  // - The prismatic joint for Seattle moves (-0.6, +0.8) for each +1.0 q₁.
  // - Dallas is not part of the robot, so its jacobian will always be zero.
  EXPECT_TRUE(CompareMatrices(clearance.jacobians(), (MatrixXd(3, 3) <<
      // Increasing either q₀ or q₁ will increase the Boston-Seattle distance.
      0.8, 0.8, 0,
      // Increasing q₀ will increase the Boston-Dallas distance.
      1.0, 0,   0,
      // Increasing q₁ will increase the Seattle-Dallas distance.
      0,   1.0, 0).finished(), tol));

  // Now reposture Seattle to be directly south of Boston (tectonic plates!).
  // The distances will change in the obvious ways; what we care about most is
  // that the jacobian and especially its sign update correctly.
  q(1) = -5.0;
  clearance = dut.CalcRobotClearance(q, influence);
  EXPECT_THAT(clearance.robot_indices(), ElementsAre(
      boston,
      boston,
      seattle));
  EXPECT_THAT(clearance.other_indices(), ElementsAre(
      seattle,
      dallas,
      dallas));
  EXPECT_THAT(clearance.collision_types(),
              ElementsAre(RobotCollisionType::kSelfCollision,
                          RobotCollisionType::kEnvironmentCollision,
                          RobotCollisionType::kEnvironmentCollision));
  // The Boston <-> Seattle distance on-center is 6.0.
  // The Boston <-> Dallas distance on-center is 5.0.
  // The Seattle <-> Dallas distance on-center is 5.0.
  // In each case, the measured distance is less than on-center by each of the
  // two cities' sphere radii.
  EXPECT_TRUE(CompareMatrices(clearance.distances(), (VectorXd(3) <<
      6.0 - 0.5 - 0.25,
      5.0 - 0.5 - 1.0,
      5.0 - 0.25 - 1.0).finished(), tol));
  // Recall that:
  // - The prismatic joint for Boston moves (+0.6, +0.8) for each +1.0 q₀.
  // - The prismatic joint for Seattle moves (-0.6, +0.8) for each +1.0 q₁.
  // - Dallas is not part of the robot, so its jacobian will always be zero.
  EXPECT_TRUE(CompareMatrices(clearance.jacobians(), (MatrixXd(3, 3) <<
      // Increasing q₀ will increase the Boston-Seattle distance.
      // Increasing q₁ will decrease the Boston-Seattle distance.
      0.6, -0.6, 0,
      // Increasing q₀ will increase the Boston-Dallas distance.
      1.0, 0,    0,
      // Increasing q₁ will decrease the Seattle-Dallas distance.
      0,   -1.0, 0).finished(), tol));
}

// Checks RobotClearance when the robot is an arm (only) but inboard of the
// arm we have a mobile chassis that is not part of a robot from the point
// of view of CollisionChecker (e.g., we're only planning for the arm).
GTEST_TEST(SceneGraphCollisionCheckerTest, ClearanceFloatingBase) {
  // Build a dut with a ground plane + floating chassis with welded arm.
  RobotDiagramBuilder<double> builder;
  builder.mutable_parser().AddModelsFromString(R"""(
directives:
- add_model:
    name: ground
    file: package://drake/planning/test_utilities/collision_ground_plane.sdf
- add_weld:
    parent: world
    child: ground::ground_plane_box
- add_model:
    name: chassis
    file: package://drake/manipulation/models/ycb/sdf/010_potted_meat_can.sdf
- add_model:
    name: arm
    file: package://drake/manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf
- add_weld:
    parent: chassis::base_link_meat
    child: arm::base
)""", "dmd.yaml");
  const auto& plant = builder.plant();
  CollisionCheckerParams params;
  params.model = builder.Build();
  params.robot_model_instances.push_back(
      plant.GetModelInstanceByName("arm"));
  params.configuration_distance_function =
      [](const VectorXd& q1, const VectorXd& q2) {
    return (q1 - q2).norm();
  };
  params.edge_step_size = 0.05;
  SceneGraphCollisionChecker dut(std::move(params));

  // In the testing below, we assume that the iiwa's floating base is
  // quaternion, and that the floating base precedes the arm in q.
  const int floating_dofs = 7;
  const int arm_dofs = 7;
  const int total_dofs = floating_dofs + arm_dofs;
  DRAKE_DEMAND(plant.num_positions() == total_dofs);
  const auto& first_arm_joint = plant.GetJointByName("iiwa_joint_1");
  DRAKE_DEMAND(first_arm_joint.position_start() == floating_dofs);

  // Check the clearance of the default posture.
  const VectorXd q = plant.GetPositions(*plant.CreateDefaultContext());
  const double influence = 99999999;
  auto clearance = dut.CalcRobotClearance(q, influence);
  EXPECT_EQ(clearance.num_positions(), total_dofs);
  EXPECT_GT(clearance.size(), 0);
  int num_non_zero = 0;
  for (int i = 0; i < clearance.size(); ++i) {
    const VectorXd jacobian = clearance.jacobians().row(i);
    ASSERT_EQ(jacobian.size(), total_dofs);
    // Non-robot dofs must always be zero.
    ASSERT_TRUE(CompareMatrices(jacobian.head(floating_dofs),
                                VectorXd::Zero(floating_dofs)));
    // Robot dofs are generally non-zero.
    if (!jacobian.tail(arm_dofs).isZero(0.0)) {
      ++num_non_zero;
    }
  }
  // At least 95% of the distances should have a non-singular jacobian. In any
  // given posture, some small number of distances will have a zero jacobian.
  // We aren't trying drill down into that here; we're only really concerned
  // with the non-robot dofs being zeroed out without the entire jacobian
  // accidentally being zeroed out.
  EXPECT_GT(static_cast<double>(num_non_zero) / clearance.size(), 0.95);
}

}  // namespace test
}  // namespace planning
}  // namespace drake

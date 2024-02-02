#include "drake/planning/iris_from_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/ssize.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"

namespace drake {
namespace planning {
namespace {
using Eigen::Vector2d;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::VPolytope;
using geometry::optimization::IrisOptions;
using geometry::optimization::VPolytope;
using geometry::Meshcat;
using geometry::Rgba;
using common::MaybePauseForUser;

/* A movable sphere in a box.
┌───────────────┐
│               │
│               │
|               │
│       o       │
│               │
│               │
│               │
└───────────────┘ */
const char free_box[] = R"""(
<robot name="boxes">
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.1"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)""";
//Test that we get perfect coverage 
GTEST_TEST(IrisInConfigurationSpaceFromCliqueCover,
           BoxConfigurationSpaceTest) {
  CollisionCheckerParams params;

  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(free_box, "urdf");
  params.edge_step_size = 0.01;

  params.model = builder.Build();
  auto checker =
      std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  IrisFromCliqueCoverOptions options;

  options.num_builders = 2;
  options.num_points_per_coverage_check = 100;
  options.num_points_per_visibility_round = 25;
  std::vector<HPolyhedron> sets;

  RandomGenerator generator;

  IrisInConfigurationSpaceFromCliqueCover(*checker, options, &generator, &sets);
  EXPECT_GE(ssize(sets), 1);

  //expect perfect coverage
  VPolytope vpoly(sets.at(0));
  EXPECT_EQ(vpoly.CalcVolume(), 16.0);
}

/* A movable sphere with fixed boxes in all corners.
┌─────┬───┬─────┐
│     │   │     │
│     │   │     │
├─────┘   └─────┤
│       o       │
├─────┐   ┌─────┤
│     │   │     │
│     │   │     │
└─────┴───┴─────┘ */
const char boxes_in_corners[] = R"""(
<robot name="boxes">
  <link name="fixed">
    <collision name="top_left">
      <origin rpy="0 0 0" xyz="-1 1 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
    <collision name="top_right">
      <origin rpy="0 0 0" xyz="1 1 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
    <collision name="bottom_left">
      <origin rpy="0 0 0" xyz="-1 -1 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
    <collision name="bottom_right">
      <origin rpy="0 0 0" xyz="1 -1 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.1"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)""";

GTEST_TEST(IrisInConfigurationSpaceFromCliqueCover,
           BoxWithCornerObstaclesTest) {
  CollisionCheckerParams params;
  
  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
  meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                             -3.25, 3.25, -3.25, 3.25);
  meshcat->SetProperty("/Grid", "visible", true);
  //draw true cspace
  Eigen::Matrix3Xd env_points = Eigen::Matrix3Xd::Zero(3, 5);
  env_points << -2, 2, 2,-2,-2,
                2, 2, -2, -2, 2,
                0, 0, 0, 0, 0;
  meshcat->SetLine("Domain", env_points, 2.0, Rgba(0, 0, 1));
  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(boxes_in_corners, "urdf");
  params.edge_step_size = 0.01;

  params.model = builder.Build();
  auto checker =
      std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  IrisFromCliqueCoverOptions options;
  
  options.num_builders = 2;
  options.num_points_per_coverage_check = 100;
  options.num_points_per_visibility_round = 25;
  std::vector<HPolyhedron> sets;

  RandomGenerator generator;

  // TODO(Alexandre.Amice) make this test actually check the cross.
  IrisInConfigurationSpaceFromCliqueCover(*checker, options, &generator, &sets);
  EXPECT_GE(ssize(sets), 2);
  Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 12);
  int region_idx = 0;
  for(auto h: sets){
    VPolytope vregion = VPolytope(h).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine(fmt::format("iris_region_{}", region_idx), points, 2.0, Rgba(0, 1, 0));
    ++region_idx;
  }
  MaybePauseForUser();
}




}  // namespace
}  // namespace planning
}  // namespace drake

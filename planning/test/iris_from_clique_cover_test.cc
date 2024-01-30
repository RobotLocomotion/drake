#include "drake/planning/iris_from_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/ssize.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace planning {
namespace {
using Eigen::Vector2d;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::IrisOptions;
using geometry::optimization::VPolytope;

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
}
}  // namespace
}  // namespace planning
}  // namespace drake

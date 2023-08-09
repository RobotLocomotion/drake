#include "drake/planning/iris_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {
namespace {

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXd;

std::unique_ptr<SceneGraphCollisionChecker>
MakeSceneGraphCollisionCheckerFromString(const std::string& file_contents,
                                         const std::string& file_type,
                                         double edge_step_size = 0.01) {
  CollisionCheckerParams params;

  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(file_contents, file_type);
  params.model = builder.Build();
  params.edge_step_size = edge_step_size;

  return std::make_unique<SceneGraphCollisionChecker>(std::move(params));
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

GTEST_TEST(IrisRegionsFromCliqueCoverTest, BoxesInCorners) {
  // Populate the points rowwise, then transpose.
  MatrixXd points(7, 2);
  // clang-format off
  points <<     0,    0,
              1.3,    0,
                0,  1.3,
             -1.3,    0,
                0, -1.3,
              1.3, -1.3,
              0.9,    0;
  // clang-format on
  points.transposeInPlace();
  auto checker =
      MakeSceneGraphCollisionCheckerFromString(boxes_in_corners, "urdf");

  /* `points` correspond to the movable sphere being in approximately the
    locations:
     ┌─────┬───┬─────┐
     │     │ 2 │     │
     │     │   │     │
     ├─────┘   └─────┤
     │ 3     0   6  1│
     ├─────┐   ┌─────┤
     │     │   │     │
     │     │ 4 │   5 │
     └─────┴───┴─────┘ */

  IrisRegionsFromCliqueCover(*checker, points,
                             geometry::optimization::IrisOptions(), 1, false);

  /*
    for (const bool parallelize : {false, true}) {

    }
  */
}

}  // namespace
}  // namespace planning
}  // namespace drake

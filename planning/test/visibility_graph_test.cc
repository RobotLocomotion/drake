#include "drake/planning/visibility_graph.h"

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
  params.distance_function_weights =
      VectorXd::Ones(params.model->plant().num_positions());
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

GTEST_TEST(VisibilityGraphTest, BoxesInCorners) {
  // Populate the points rowwise, then transpose.
  MatrixXd points(5, 2);
  // clang-format off
  points <<     0,    0,
              1.8,    0,
                0,  1.8,
             -1.8,    0,
                0, -1.8;
  // clang-format on
  points.transposeInPlace();

  /* `points` correspond to the movable sphere being in approximately the
    locations:
     ┌─────┬───┬─────┐
     │     │ 2 │     │
     │     │   │     │
     ├─────┘   └─────┤
     │ 3     0     1 │
     ├─────┐   ┌─────┤
     │     │   │     │
     │     │ 4 │     │
     └─────┴───┴─────┘ */

  for (const bool parallelize : {false, true}) {
    auto checker =
        MakeSceneGraphCollisionCheckerFromString(boxes_in_corners, "urdf");
    // Confirm that the midpoint between 1 and 2 is in collision.
    EXPECT_FALSE(checker->CheckConfigCollisionFree(
        checker->InterpolateBetweenConfigurations(points.col(1), points.col(2),
                                                  0.5)));

    SparseMatrix<bool> A = VisibilityGraph(*checker, points, parallelize);

    MatrixX<bool> A_expected = MatrixX<bool>::Zero(5, 5);  // All false.
    A_expected(0, 1) = true;
    A_expected(0, 2) = true;
    A_expected(0, 3) = true;
    A_expected(0, 4) = true;
    A_expected(1, 3) = true;
    A_expected(2, 4) = true;

    // Fill in the upper-triangular portion (to make the matrix symmetric)
    A_expected += A_expected.transpose().eval();

    // Note: cast to int because CompareMatrices doesn't like bool.
    EXPECT_TRUE(
        CompareMatrices(A.toDense().cast<int>(), A_expected.cast<int>()));

    // Now with a (too) large edge step size, it returns a fully connected
    // graph.
    const double kLargeEdgeStepSize = 10.0;
    checker = MakeSceneGraphCollisionCheckerFromString(boxes_in_corners, "urdf",
                                                       kLargeEdgeStepSize);

    A = VisibilityGraph(*checker, points, parallelize);
    A_expected = MatrixX<bool>::Constant(5, 5, true);
    for (int i = 0; i < 5; ++i) {
      A_expected(i, i) = false;
    }
    EXPECT_TRUE(
        CompareMatrices(A.toDense().cast<int>(), A_expected.cast<int>()));
  }
}

// Confirm that points in collision are not connected to anything, even if the
// edge step size is large.
GTEST_TEST(VisibilityGraphTest, BoxesInCorners2) {
  const double kLargeEdgeStepSize = 10.0;
  auto checker = MakeSceneGraphCollisionCheckerFromString(
      boxes_in_corners, "urdf", kLargeEdgeStepSize);

  MatrixXd points(2, 2);
  // clang-format off
  points <<  0, 0,  // collision-free
             1, 1;  // in collision.
  // clang-format on
  points.transposeInPlace();

  EXPECT_TRUE(checker->CheckConfigCollisionFree(points.col(0)));
  EXPECT_FALSE(checker->CheckConfigCollisionFree(points.col(1)));

  SparseMatrix<bool> A = VisibilityGraph(*checker, points);
  // There is no edge from point 0 to point 1.
  EXPECT_FALSE(A.coeffRef(0, 1));

  // This is still true if it's the first point that is in collision.
  MatrixXd shuffled_points(2, 2);
  shuffled_points << points.col(1), points.col(0);
  A = VisibilityGraph(*checker, shuffled_points);
  EXPECT_FALSE(A.coeffRef(0, 1));
}

}  // namespace
}  // namespace planning
}  // namespace drake

#include "drake/planning/iris_regions_from_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace planning {
namespace {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Vector2d;
using Eigen::VectorXd;
using geometry::Meshcat;

std::unique_ptr<SceneGraphCollisionChecker>
MakeSceneGraphCollisionCheckerFromString(const std::string& file_contents,
                                         const std::string& file_type,
                                         std::shared_ptr<Meshcat> meshcat) {
  CollisionCheckerParams params;

  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(file_contents, file_type);
  builder.plant().Finalize();
  visualization::AddDefaultVisualization(&builder.builder(), meshcat);
  params.model = builder.Build();
  params.edge_step_size = 0.01;

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
  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
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
  auto checker = MakeSceneGraphCollisionCheckerFromString(boxes_in_corners,
                                                          "urdf", meshcat);

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

/* A movable sphere with obstacles forming a five-point star configuration
 * space. */
GTEST_TEST(IrisRegionsFromCliqueCoverTest, FivePointStar) {
  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
  meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                           -3.25, 3.25, -3.25, 3.25);

  const double alpha = 2.0 * M_PI / 5.0;
  const double beta = M_PI / 10.0;
  const Matrix2d R_alpha =
      (Matrix2d() << cos(alpha), -sin(alpha), sin(alpha), cos(alpha))
          .finished();
  const Matrix2d R_beta =
      (Matrix2d() << cos(beta), -sin(beta), sin(beta), cos(beta)).finished();
  const Vector2d corner{2.0 * sin(beta) * cos(3.0 * M_PI / 10.0),
                        2.0 * sin(beta) * sin(3.0 * M_PI / 10.0)};
  Vector2d center = corner + Vector2d::Ones();
  Vector2d center2 = corner + R_beta * Vector2d::Ones();

  std::string five_point_star = R"""(
<robot name="boxes">
  <link name="fixed">
)""";

  for (int i = 0; i < 5; ++i) {
    five_point_star += fmt::format(
        R"""(
    <collision name="corner{}">
      <origin rpy="0 0 {}" xyz="{} {} 0"/>
      <geometry><box size="2 2 2"/></geometry>
    </collision>
    <collision name="corner{}_2">
      <origin rpy="0 0 {}" xyz="{} {} 0"/>
      <geometry><box size="2 2 2"/></geometry>
    </collision>
)""",
        i, i * alpha, center[0], center[1], i, i * alpha + beta, center2[0],
        center2[1]);
    center = R_alpha * center;
    center2 = R_alpha * center2;
  }

  five_point_star += R"""(
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.05"/></geometry>
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

  auto checker = MakeSceneGraphCollisionCheckerFromString(five_point_star,
                                                          "urdf", meshcat);
  checker->model().ForcedPublish(checker->model_context().model_context());
  meshcat->SetProperty("proximity", "visible", true);

  // Populate the points rowwise, then transpose.
  MatrixXd points(7, 2);
  // clang-format off
  points <<   0.0,  0.0,
              1.3,  0.42,
              0.0,  1.3,
             -1.3,  0.42,
             -0.8, -1.1,
              0.8, -1.1;
  // clang-format on
  points.transposeInPlace();
  for (int i=0; i<points.cols(); ++i) {
    std::string path = fmt::format("Point{}", i);
    meshcat->SetObject(path, geometry::Sphere(0.05), geometry::Rgba(0, 1, 0));
    meshcat->SetTransform(path, math::RigidTransform(Eigen::Vector3d(
                                    points(0, i), points(1, i), 0)));
  }
  common::MaybePauseForUser();

  logging::set_log_level("debug");
  IrisRegionsFromCliqueCover(*checker, points,
                             geometry::optimization::IrisOptions(), 1, false);

}

}  // namespace
}  // namespace planning
}  // namespace drake

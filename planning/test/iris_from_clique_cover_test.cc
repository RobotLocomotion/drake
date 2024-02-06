#include "drake/planning/iris_from_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/ssize.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace planning {
namespace {
using common::MaybePauseForUser;
using Eigen::Vector2d;
using geometry::Meshcat;
using geometry::Rgba;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperrectangle;
using geometry::optimization::IrisOptions;
using geometry::optimization::VPolytope;

// Draw a
void Draw2dVPolytope(const VPolytope& polytope, const std::string& meshcat_name,
                     const Eigen::Ref<const Eigen::Vector3d>& color,
                     std::shared_ptr<Meshcat> meshcat) {
  DRAKE_THROW_UNLESS(polytope.ambient_dimension() == 2);
  Eigen::Matrix3Xd points =
      Eigen::Matrix3Xd::Zero(3, polytope.vertices().cols() + 1);
  points.topLeftCorner(2, polytope.vertices().cols()) = polytope.vertices();
  points.topRightCorner(2, 1) = polytope.vertices().col(0);
  points.bottomRows<1>().setZero();

  meshcat->SetLine(meshcat_name, points, 2.0,
                   Rgba(color(0), color(1), color(2)));
}

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
// Test that we get perfect coverage
GTEST_TEST(IrisInConfigurationSpaceFromCliqueCover, BoxConfigurationSpaceTest) {
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

  // expect perfect coverage
  VPolytope vpoly(sets.at(0));
  EXPECT_EQ(vpoly.CalcVolume(), 16.0);
}

/* A movable sphere with fixed boxes in all corners.
┌───────────────┐
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
│       o       │
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
└───────────────┘ */
const char boxes_in_corners[] = R"""(
<robot name="boxes">
  <link name="fixed">
    <collision name="top_left">
      <origin rpy="0 0 0" xyz="-1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="top_right">
      <origin rpy="0 0 0" xyz="1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_left">
      <origin rpy="0 0 0" xyz="-1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_right">
      <origin rpy="0 0 0" xyz="1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.01"/></geometry>
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
  // draw true cspace
  Eigen::Matrix3Xd env_points = Eigen::Matrix3Xd::Zero(3, 5);
  env_points << -2, 2, 2, -2, -2, 2, 2, -2, -2, 2, 0, 0, 0, 0, 0;
  meshcat->SetLine("Domain", env_points, 8.0, Rgba(0, 0, 0));
  Eigen::Matrix3Xd centers = Eigen::Matrix3Xd::Zero(3, 4);
  double c = 1.0;
  centers << -c, c, c, -c, c, c, -c, -c, 0, 0, 0, 0;
  Eigen::Matrix3Xd obs_points = Eigen::Matrix3Xd::Zero(3, 5);
  // approximating offset due to sphere radius with fixed offset
  double s = 0.7 + 0.01;
  obs_points << -s, s, s, -s, -s, s, s, -s, -s, s, 0, 0, 0, 0, 0;
  for (int obstacle_idx = 0; obstacle_idx < 4; ++obstacle_idx) {
    Eigen::Matrix3Xd obstacle = obs_points;
    obstacle.colwise() += centers.col(obstacle_idx);
    meshcat->SetLine(fmt::format("/obstacles/obs_{}", obstacle_idx), obstacle,
                     8.0, Rgba(0, 0, 0));
  }

  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(boxes_in_corners, "urdf");
  params.edge_step_size = 0.01;

  params.model = builder.Build();
  auto checker =
      std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  IrisFromCliqueCoverOptions options;
  options.iris_options.meshcat = meshcat;

  options.num_builders = 2;
  options.num_points_per_coverage_check = 1000;
  options.num_points_per_visibility_round = 100;
  options.coverage_termination_threshold = 0.95;
  std::vector<HPolyhedron> sets;

  RandomGenerator generator;

  // TODO(Alexandre.Amice) make this test actually check the cross.
  IrisInConfigurationSpaceFromCliqueCover(*checker, options, &generator, &sets);
  EXPECT_EQ(ssize(sets), 6);

  //   A manual convex decomposition of the space.
  std::vector<Hyperrectangle> manual_decomposition{
      Hyperrectangle(Vector2d{-2, -2}, Vector2d{-1.7, 2}),
      Hyperrectangle(Vector2d{-2, -2}, Vector2d{2, -1.7}),
      Hyperrectangle(Vector2d{1.7, -2}, Vector2d{2, 2}),
      Hyperrectangle(Vector2d{-2, 1.7}, Vector2d{2, 2}),
      Hyperrectangle(Vector2d{-0.3, -2}, Vector2d{0.3, 2}),
      Hyperrectangle(Vector2d{-2, -0.3}, Vector2d{2, 0.3}),
  };

  // Show the manual decomposition in the meshcat debugger.
  Eigen::VectorXd color(3);
  std::normal_distribution<double> gaussian;
  for (int i = 0; i < ssize(manual_decomposition); ++i) {
    // Choose a random color.
    for (int j = 0; j < color.size(); ++j) {
      color[j] = abs(gaussian(generator));
    }
    color.normalize();
    VPolytope vregion = VPolytope(manual_decomposition.at(i).MakeHPolyhedron())
                            .GetMinimalRepresentation();
    Draw2dVPolytope(vregion, fmt::format("manual_decomposition_{}", i), color,
                    meshcat);
  }

  // Show the IrisFromCliqueCoverDecomposition
  for (int i = 0; i < ssize(sets); ++i) {
    // Choose a random color.
    for (int j = 0; j < color.size(); ++j) {
      color[j] = abs(gaussian(generator));
    }
    color.normalize();
    VPolytope vregion = VPolytope(sets.at(i)).GetMinimalRepresentation();
    Draw2dVPolytope(vregion, fmt::format("iris_from_clique_cover_{}", i),
                    color, meshcat);
  }

  // Now check the coverage by drawing points from the manual decomposition and
  // checking if they are inside the IrisFromCliqueCover decomposition.
  int num_samples_per_set = 1000;
  int num_in_automatic_decomposition = 0;
  for(const auto& manual_set: manual_decomposition) {
    for(int i = 0; i < num_samples_per_set; ++i) {
     Eigen::Vector2d sample = manual_set.UniformSample(&generator);
     for(const auto& set: sets) {
       if(set.PointInSet(sample)) {
         ++num_in_automatic_decomposition;
         break;
       }
     }
    }
  }
  double coverage_estimate = static_cast<double>(num_in_automatic_decomposition) /
                             static_cast<double>(num_samples_per_set * ssize(manual_decomposition));
  EXPECT_GE(coverage_estimate, 0.9);

  MaybePauseForUser();
}

}  // namespace
}  // namespace planning
}  // namespace drake

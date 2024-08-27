#include "drake/planning/iris/iris_entrypoint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {
namespace {

using Eigen::VectorXd;

/* A movable sphere in a box.
┌───────────────┐
│               │
│               │
│               │
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

std::unique_ptr<CollisionChecker> MakeSphereInCollisionChecker() {
  CollisionCheckerParams params;

  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(free_box, "urdf");
  params.edge_step_size = 0.01;

  params.model = builder.Build();
  auto checker =
      std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  return checker;
}

std::vector<IrisAlgorithm> all_algorithms = {
  IrisAlgorithm::Convex,
  IrisAlgorithm::NP,
  IrisAlgorithm::NP2,
  IrisAlgorithm::ZO,
  IrisAlgorithm::Certified,
};

std::vector<IrisRegionSpace> all_region_spaces = {
  IrisRegionSpace::TaskSpace2d,
  IrisRegionSpace::TaskSpace3d,
  IrisRegionSpace::AbstractSpaceNd,
  IrisRegionSpace::ConfigurationSpace,
  IrisRegionSpace::RationalConfigurationSpace,
};

GTEST_TEST(IrisEntrypointTest, NotImplemented) {
  std::vector<IrisOptions> not_implemented_option();

  // Not yet implemented.
  for (const auto& algorithm : all_algorithms) {
    if (algorithm != IrisAlgorithm::NP) {
      IrisOptions options;
      options.algorithm = algorithm;
      options.region_space = IrisRegionSpace::TaskSpace2d;
      not_implemented_option.push_back(options);
    }
  }

  std::unique_ptr<CollisionChecker> checker = MakeSphereInCollisionChecker();
  for (const auto& invalid_option : not_implemented_option) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        GrowIrisRegion(*checker, invalid_option, VectorXd::Zero(2)),
        ".*not supported yet.*");
  }
}

// GTEST_TEST(IrisEntrypointTest, UnsupportedOptionsNP) {
//   std::vector<IrisAlgorithm>
// }

}  // namespace
}  // namespace planning
}  // namespace drake

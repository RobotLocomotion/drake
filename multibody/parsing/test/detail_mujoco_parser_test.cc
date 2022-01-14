#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::SceneGraph;

GTEST_TEST(MujocoParser, TestGymModels) {
  // Confirm that I can successfully parse the MuJoCo models in the
  // DeepMind control suite.
  for (const auto& f :
       {"acrobot", "cartpole", "cheetah", "finger", "fish", "hopper",
        "humanoid", "humanoid_CMU", "lqr", "manipulator", "pendulum",
        "point_mass", "reacher", "stacker", "swimmer", "walker"}) {
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);

    const std::string filename = FindResourceOrThrow(
        fmt::format("drake/multibody/parsing/dm_control/suite/{}.xml", f));
    AddModelFromMujocoXml({.file_name = &filename}, f, std::nullopt, &plant);

    EXPECT_TRUE(plant.HasModelInstanceNamed(f));
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

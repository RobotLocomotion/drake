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
  // OpenAI Gym.
  for (const auto& f :
       {"ant", "half_cheetah", "hopper", "humanoid", "humanoidstandup",
        "inverted_double_pendulum", "inverted_pendulum", "point", "pusher",
        "reacher", "striker", "thrower", "walker2d"}) {
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);

    const std::string filename = FindResourceOrThrow(
        fmt::format("drake/multibody/models/mujoco_gym/{}.xml", f));
    AddModelFromMujocoXml({.file_name = &filename}, f, std::nullopt, &plant);

    EXPECT_TRUE(plant.HasModelInstanceNamed(f));
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

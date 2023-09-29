#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(ParseTest, SpheresCollision) {
  multibody::MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  multibody::Parser parser(&plant);
  parser.AddModels(FindResourceOrThrow(
      "drake/manipulation/models/z1_description/xacro/z1.urdf"));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link00"));
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 6);
}


}  // namespace
}  // namespace manipulation
}  // namespace drake

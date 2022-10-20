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
  parser.AddAllModelsFromFile(FindResourceOrThrow(
      "drake/manipulation/models/ur3e/ur3e_spheres_collision.urdf"));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ur_base_link"));
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 6);
}

GTEST_TEST(ParseTest, CylindersCollision) {
  multibody::MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  multibody::Parser parser(&plant);
  parser.AddAllModelsFromFile(FindResourceOrThrow(
      "drake/manipulation/models/ur3e/ur3e_cylinders_collision.urdf"));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ur_base_link"));
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 6);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

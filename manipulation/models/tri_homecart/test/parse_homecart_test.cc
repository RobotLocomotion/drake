#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(ParseHomecartTest, BasicTest) {
  multibody::MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  multibody::Parser parser(&plant);
  multibody::parsing::ModelDirectives directives =
      multibody::parsing::LoadModelDirectives(FindResourceOrThrow(
          "drake/manipulation/models/tri_homecart/homecart.dmd.yaml"));
  multibody::parsing::ProcessModelDirectives(directives, &plant, nullptr,
                                             &parser);
  plant.Finalize();

  // 6 positions per ur3e and 2 per wsg.
  EXPECT_EQ(plant.num_positions(), 6 + 2 + 6 + 2);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

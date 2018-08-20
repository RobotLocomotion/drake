#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
// #include "drake/multibody/parsers/sdf_parser.h"
// #include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(AllegroHandTest, TestTree) {
  const std::string kPathRight(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/sdf/"
      "allegro_hand_description_right.sdf"));
  const std::string kPathLeft(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/sdf/"
      "allegro_hand_description_left.sdf"));

  systems::DiagramBuilder<double> builder;
  multibody::multibody_plant::MultibodyPlant<double>& plant =
      *builder.AddSystem<multibody::multibody_plant::MultibodyPlant>();
  multibody::parsing::AddModelFromSdfFile(
                          kPathRight, &plant, nullptr);
  EXPECT_EQ(plant.num_actuators(), 16);
  EXPECT_EQ(plant.num_actuated_dofs(), 16);

  multibody::multibody_plant::MultibodyPlant<double>& plant_left =
      *builder.AddSystem<multibody::multibody_plant::MultibodyPlant>();
  multibody::parsing::AddModelFromSdfFile(
                          kPathRight, &plant_left, nullptr);
  EXPECT_EQ(plant_left.num_actuators(), 16);
  EXPECT_EQ(plant_left.num_actuated_dofs(), 16);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

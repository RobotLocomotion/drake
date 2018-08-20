#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

namespace drake {
namespace manipulation {
namespace {

using multibody::multibody_plant::MultibodyPlant;
using multibody::ModelInstanceIndex;
using multibody::parsing::AddModelFromSdfFile;

GTEST_TEST(AllegroHandTest, TestTree) {
  const std::string kPathRight(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/sdf/"
      "allegro_hand_description_right.sdf"));
  const std::string kPathLeft(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/sdf/"
      "allegro_hand_description_left.sdf"));

  MultibodyPlant<double> plant;
  const ModelInstanceIndex right_hand_index = AddModelFromSdfFile(
                                                      kPathRight, &plant);
  const ModelInstanceIndex left_hand_index = AddModelFromSdfFile(
                                                      kPathLeft, &plant);
  plant.Finalize();

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for each hand in an SDF file
  EXPECT_EQ(plant.num_model_instances(), 4);

  EXPECT_EQ(plant.num_joints(), 2 * 16);
  EXPECT_EQ(plant.num_actuators(), 2 * 16);
  // 16 for fingers, 1 for hand root, 1 for world
  EXPECT_EQ(plant.num_bodies(), 2 * 17 + 1);
  // 16 for finger joints, 7 for the free moving hand in the space
  EXPECT_EQ(plant.num_positions(right_hand_index), 23);
  EXPECT_EQ(plant.num_velocities(right_hand_index), 22);
  EXPECT_EQ(plant.num_positions(left_hand_index), 23);
  EXPECT_EQ(plant.num_velocities(left_hand_index), 22);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

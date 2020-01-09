#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

using multibody::MultibodyPlant;
using multibody::ModelInstanceIndex;
using multibody::Parser;

class ParseTest : public testing::TestWithParam<std::string> {};

TEST_P(ParseTest, Quantities) {
  const std::string file_extension = GetParam();
  const std::string path_right = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/allegro_hand_description/{}/"
      "allegro_hand_description_right.{}", file_extension, file_extension));
  const std::string path_left = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/allegro_hand_description/{}/"
      "allegro_hand_description_left.{}", file_extension, file_extension));

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  const ModelInstanceIndex right_hand_index =
      parser.AddModelFromFile(path_right);
  const ModelInstanceIndex left_hand_index =
      parser.AddModelFromFile(path_left);
  plant.Finalize();

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for each hand in an SDF file
  EXPECT_EQ(plant.num_model_instances(), 4);

  EXPECT_EQ(plant.num_actuators(), 2 * 16);
  if (file_extension == "sdf") {
    EXPECT_EQ(plant.num_joints(), 2 * 16);
    EXPECT_EQ(plant.num_bodies(), 2 * 17 + 1);  // + 1 for world
  } else {
    EXPECT_EQ(plant.num_joints(), 2 * 21);
    EXPECT_EQ(plant.num_bodies(), 2 * 22 + 1);  // + 1 for world
  }
  // 16 for finger joints, 7 for the free moving hand in the space
  EXPECT_EQ(plant.num_positions(right_hand_index), 23);
  EXPECT_EQ(plant.num_velocities(right_hand_index), 22);
  EXPECT_EQ(plant.num_positions(left_hand_index), 23);
  EXPECT_EQ(plant.num_velocities(left_hand_index), 22);
}

INSTANTIATE_TEST_SUITE_P(Both, ParseTest, testing::Values("sdf", "urdf"));

}  // namespace
}  // namespace manipulation
}  // namespace drake

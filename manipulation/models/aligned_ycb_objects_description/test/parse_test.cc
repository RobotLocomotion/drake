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
  const std::string object_name = GetParam();
  const std::string kPath(FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/aligned_ycb_objects_description/sdf/"
      "{}.sdf", object_name)));

  multibody::MultibodyPlant<double> plant;
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);
  plant.Finalize();

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for each YCB sdf file.
  EXPECT_EQ(plant.num_model_instances(), 3);

  // Each object has two bodies, the world body and the object body.
  EXPECT_EQ(plant.num_bodies(), 2);
}

INSTANTIATE_TEST_CASE_P(Both, ParseTest, testing::Values(
    "cracker_box",
    "gelatin_box",
    "mustard_bottle",
    "potted_meat_can",
    "soup_can",
    "sugar_box"));

}  // namespace
}  // namespace manipulation
}  // namespace drake

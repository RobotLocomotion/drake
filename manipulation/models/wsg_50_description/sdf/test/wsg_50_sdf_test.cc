#include <iostream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that all the Sdf files can be parsed into a plant.
GTEST_TEST(Wsg50DescriptionSdfTest, TestBoxSchunkSdf) {
  const std::string prefix("drake/manipulation/models/wsg_50_description/sdf/");
  const std::vector<std::string> file_names{
    "schunk_wsg_50.sdf",
    "schunk_wsg_50_ball_contact.sdf",
    "schunk_wsg_50_no_tip.sdf",
    "schunk_wsg_50_with_tip.sdf"};

  for (size_t i = 0; i < file_names.size(); i++) {
    const std::string kPath(FindResourceOrThrow(prefix + file_names[i]));

    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModelFromFile(kPath);
    plant.Finalize();

    // Expect 3 model instances. One for the world, one default model instance
    // for unspecified modeling elements, and the gripper model instance.
    EXPECT_EQ(plant.num_model_instances(), 3);

    // Expect 9 positions, 6 attached to the body, 2 to each joints of the
    // gripper, and the first one reserved for multibody plant.
    EXPECT_EQ(plant.num_positions(), 9);

    // Expect 8 velocities, 6 attached to the body, 2 to each joints of the
    // gripper.
    EXPECT_EQ(plant.num_velocities(), 8);

    ASSERT_EQ(plant.num_bodies(), 4);
    const std::vector<std::string> expected_body_names{
      "" /* Ignore the world body name */,
        "body",
        "left_finger",
        "right_finger"};
    for (multibody::BodyIndex j{1}; j < plant.num_bodies(); ++j) {
      EXPECT_THAT(plant.get_body(j).name(),
                  testing::EndsWith(expected_body_names.at(j)));
    }
  }
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

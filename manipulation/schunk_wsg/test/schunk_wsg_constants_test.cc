#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

// Test that the constants defined in schunk_wsg_constants.h are
// correct wrt `models/schunk_wsg_50.sdf`.
GTEST_TEST(SchunkWsgConstantTest, ConstantTest) {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body"));
  plant.Finalize();

  EXPECT_EQ(plant.num_actuators(), kSchunkWsgNumActuators);
  EXPECT_EQ(plant.num_positions(), kSchunkWsgNumPositions);
  EXPECT_EQ(plant.num_velocities(), kSchunkWsgNumVelocities);

  const auto& joint = plant.GetJointByName("left_finger_sliding_joint");
  const auto& matrix = plant.MakeStateSelectorMatrix({joint.index()});
  EXPECT_EQ(matrix(0, kSchunkWsgPositionIndex), 1.0);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

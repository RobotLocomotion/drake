#include "drake/multibody/rigid_body_plant/rigid_body_plant_that_publishes_xdot.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/multibody/parsers/urdf_parser.h"

using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;

namespace drake {

using lcm::CompareLcmtDrakeSignalMessages;
using lcm::DrakeMockLcm;
using multibody::joints::kQuaternion;
using parsers::urdf::AddModelInstanceFromUrdfFile;
using systems::Context;
using systems::RigidBodyPlantThatPublishesXdot;

namespace multibody {
namespace rigid_body_plant {
namespace test {
namespace {

// Tests that RigidBodyPlantThatPublishesXdot actually publishes LCM messages
// when DoPublish() is called.
GTEST_TEST(RigidBodyPlantThatPublishesXdotTest, TestPublishLcmMessage) {
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/multibody/models/box.urdf",
      kQuaternion, nullptr /* weld to frame */, tree_ptr.get());
  string kChannelName = "XDOT_CHANNEL_NAME";
  DrakeMockLcm lcm;
  RigidBodyPlantThatPublishesXdot<double> dut(move(tree_ptr), kChannelName,
      &lcm);
  // A quaternion floating joint has 7 position values and 6 velocity values.
  const int kNumStates = 13;

  // Verifies that the number of states, inputs, and outputs are correct.
  EXPECT_EQ(dut.get_num_states(), kNumStates);
  EXPECT_EQ(dut.get_input_size(), 0);
  EXPECT_EQ(dut.get_output_size(), kNumStates);

  // Forces the DUT to publish an LCM message.
  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  EXPECT_NO_THROW(dut.Publish(*context));

  // Verifies that the transmitted message is correct.
  const lcmt_drake_signal transmitted_message =
      lcm.DecodeLastPublishedMessageAs<lcmt_drake_signal>(kChannelName);

  lcmt_drake_signal expected_message;
  expected_message.dim = kNumStates;
  expected_message.val = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9.81};
  expected_message.coord = {"base_x", "base_y", "base_z", "base_qw", "base_qx",
                            "base_qy", "base_qz", "base_wx", "base_wy",
                            "base_wz", "base_vx", "base_vy", "base_vz"};
  expected_message.timestamp = 0;

  // Compares the previously decoded message entry by entry.
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(
    transmitted_message, expected_message));
}

}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace multibody
}  // namespace drake

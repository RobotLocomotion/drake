#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

GTEST_TEST(IiwaConstantsTest, ParseIiwaControlMode) {
  EXPECT_TRUE(position_enabled(IiwaControlMode::kPositionAndTorque) &&
              position_enabled(IiwaControlMode::kPositionOnly) &&
              !position_enabled(IiwaControlMode::kTorqueOnly));
  EXPECT_TRUE(torque_enabled(IiwaControlMode::kPositionAndTorque) &&
              torque_enabled(IiwaControlMode::kTorqueOnly) &&
              !torque_enabled(IiwaControlMode::kPositionOnly));
  EXPECT_EQ(ParseIiwaControlMode("position_only"),
            IiwaControlMode::kPositionOnly);
  EXPECT_EQ(ParseIiwaControlMode("torque_only"), IiwaControlMode::kTorqueOnly);
  EXPECT_EQ(ParseIiwaControlMode("position_and_torque"),
            IiwaControlMode::kPositionAndTorque);
  EXPECT_THROW(ParseIiwaControlMode("asdf"), std::runtime_error);
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

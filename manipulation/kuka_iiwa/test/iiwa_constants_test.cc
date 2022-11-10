#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

GTEST_TEST(IiwaConstantsTest, ParseIiwaControlMode) {
  EXPECT_TRUE(IsValid(IiwaControlMode::kDefault));
  EXPECT_FALSE(IsValid(IiwaControlMode{}));
  EXPECT_EQ(ParseIiwaControlMode({"position"}), IiwaControlMode::kPosition);
  EXPECT_EQ(ParseIiwaControlMode({"torque"}), IiwaControlMode::kTorque);
  EXPECT_EQ(
      ParseIiwaControlMode({"position", "torque"}),
      IiwaControlMode::kPosition | IiwaControlMode::kTorque);
  EXPECT_THROW(ParseIiwaControlMode({"asdf"}), std::runtime_error);
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

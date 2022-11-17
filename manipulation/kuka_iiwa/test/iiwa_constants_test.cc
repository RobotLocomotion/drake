#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

GTEST_TEST(IiwaConstantsTest, ParseIiwaControlMode) {
  EXPECT_TRUE(IsValid(IiwaControlMode::Default));
  EXPECT_FALSE(IsValid(IiwaControlMode{}));
  EXPECT_EQ(ParseIiwaControlMode({"position"}), IiwaControlMode::Position);
  EXPECT_EQ(ParseIiwaControlMode({"torque"}), IiwaControlMode::Torque);
  EXPECT_EQ(
      ParseIiwaControlMode({"position", "torque"}),
      IiwaControlMode::Position | IiwaControlMode::Torque);
  EXPECT_THROW(ParseIiwaControlMode({"asdf"}), std::runtime_error);
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

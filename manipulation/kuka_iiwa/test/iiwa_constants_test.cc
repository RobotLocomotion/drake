#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

GTEST_TEST(IiwaConstantsTest, ParseIiwaControlMode) {
  EXPECT_EQ(ParseIiwaControlMode({"position"}), kIiwaPositionMode);
  EXPECT_EQ(ParseIiwaControlMode({"torque"}), kIiwaTorqueMode);
  EXPECT_EQ(
      ParseIiwaControlMode({"position", "torque"}),
      kIiwaPositionMode | kIiwaTorqueMode);
  EXPECT_THROW(ParseIiwaControlMode({"asdf"}), std::runtime_error);
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

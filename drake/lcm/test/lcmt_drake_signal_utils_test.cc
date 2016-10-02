#include "drake/lcm/lcmt_drake_signal_utils.h"

#include "gtest/gtest.h"

#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

// This is a test fixture.
class LcmtDrakeSignalUtilsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    message1_.dim = 2;
    message1_.val.push_back(0.3739558136);
    message1_.val.push_back(0.2801694990);
    message1_.coord.push_back("artin's constant");
    message1_.coord.push_back("bernstein's constant");
    message1_.timestamp = 142857;
  }

  drake::lcmt_drake_signal message1_;
};

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are equal.
TEST_F(LcmtDrakeSignalUtilsTest, TestMatch) {
  drake::lcmt_drake_signal message2;
  message2 = message1_;

  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(message1_, message2));
}

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are unequal becuase their dimensions
// don't match.
TEST_F(LcmtDrakeSignalUtilsTest, TestDimensionsMismatch) {
  drake::lcmt_drake_signal message2;
  message2 = message1_;

  message2.dim = 1;
  message2.val.pop_back();
  message2.coord.pop_back();

  EXPECT_FALSE(CompareLcmtDrakeSignalMessages(message1_, message2));
}

}  // namespace
}  // namespace lcm
}  // namespace drake

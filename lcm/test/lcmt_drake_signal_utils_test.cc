#include "drake/lcm/lcmt_drake_signal_utils.h"

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

// This is a test fixture.
class LcmtDrakeSignalUtilsTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmtDrakeSignalUtilsTest)
  LcmtDrakeSignalUtilsTest() = default;

 protected:
  void SetUp() override {
    message1_.dim = 2;
    message1_.val.push_back(0.3739558136);
    message1_.val.push_back(0.2801694990);
    message1_.coord.push_back("artin's constant");
    message1_.coord.push_back("bernstein's constant");
    message1_.timestamp = 142857;

    message2_ = message1_;
  }

  drake::lcmt_drake_signal message1_{};
  drake::lcmt_drake_signal message2_{};
};

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are equal.
TEST_F(LcmtDrakeSignalUtilsTest, TestMatch) {
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(message1_, message2_));
}

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are unequal because their dimensions
// don't match.
TEST_F(LcmtDrakeSignalUtilsTest, TestDimensionsMismatch) {
  message2_.dim = 1;
  message2_.val.pop_back();
  message2_.coord.pop_back();

  EXPECT_FALSE(CompareLcmtDrakeSignalMessages(message1_, message2_));
}

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are unequal because their timestamps
// don't match.
TEST_F(LcmtDrakeSignalUtilsTest, TestTimestampsMismatch) {
  message2_.timestamp = 4;
  EXPECT_FALSE(CompareLcmtDrakeSignalMessages(message1_, message2_));
}

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are unequal because their values
// don't match.
TEST_F(LcmtDrakeSignalUtilsTest, TestValMismatch) {
  message2_.val[0] = 1980;
  EXPECT_FALSE(CompareLcmtDrakeSignalMessages(message1_, message2_));
}

// Tests CompareLcmtDrakeSignalMessages()'s ability to determine that two
// `drake::lcmt_drake_signal` messages are unequal because their coordinates
// don't match.
TEST_F(LcmtDrakeSignalUtilsTest, TestCoordMismatch) {
  message2_.coord[0] = "foo";
  EXPECT_FALSE(CompareLcmtDrakeSignalMessages(message1_, message2_));
}

}  // namespace
}  // namespace lcm
}  // namespace drake

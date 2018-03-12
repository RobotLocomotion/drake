/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/regions.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

GTEST_TEST(SRangeTest, DefaultConstructionAndAccessors) {
  const SRange dut;
  EXPECT_EQ(dut.s0(), 0.);
  EXPECT_EQ(dut.s1(), 0.);
}

GTEST_TEST(SRangeTest, NondefaultConstructionAndAccessors) {
  {
    const SRange dut(10., 50.);
    EXPECT_EQ(dut.s0(), 10.);
    EXPECT_EQ(dut.s1(), 50.);
  }
  // Inverted order is allowed and preserved:
  {
    SRange dut(79., 23.);
    EXPECT_EQ(dut.s0(), 79.);
    EXPECT_EQ(dut.s1(), 23.);
  }
}

GTEST_TEST(SRangeTest, Setters) {
  SRange dut;
  dut.set_s0(26.);
  dut.set_s1(-90.);
  EXPECT_EQ(dut.s0(), 26.);
  EXPECT_EQ(dut.s1(), -90.);
}

GTEST_TEST(SRangeTest, Copying) {
  const SRange source(12., 24.);
  const SRange dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(SRangeTest, Assignment) {
  const SRange source(12., 24.);
  SRange dut;
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


GTEST_TEST(LaneSRangeTest, ConstructionAndAccessors) {
  LaneSRange dut(LaneId("dut"), SRange(34., 0.));
  EXPECT_EQ(dut.lane_id(), LaneId("dut"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.s_range(), SRange(34., 0.)));

  // Exercise convenient construction via initializer list for s_range.
  EXPECT_NO_THROW(LaneSRange(LaneId("dut"), {0., 50.}));
}

GTEST_TEST(LaneSRangeTest, Copying) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  const LaneSRange dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(LaneSRangeTest, Assignment) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  LaneSRange dut(LaneId("yyy"), SRange(40., 99.));  // e.g., "something else"
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


class LaneSRouteTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_.emplace_back(LaneId("id1"), SRange(0., 10.));
    source_.emplace_back(LaneId("id2"), SRange(90., 17.));
  }

  std::vector<LaneSRange> source_;
};

TEST_F(LaneSRouteTest, DefaultConstructionAndAccessors) {
  const LaneSRoute dut;
  EXPECT_TRUE(dut.ranges().empty());
}

TEST_F(LaneSRouteTest, NondefaultConstructionAndAccessors) {
  const LaneSRoute dut(source_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.ranges(), source_));
}

TEST_F(LaneSRouteTest, Copying) {
  const LaneSRoute dut_source(source_);
  const LaneSRoute dut(dut_source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, dut_source));
}

TEST_F(LaneSRouteTest, Assignment) {
  const LaneSRoute dut_source(source_);
  LaneSRoute dut;
  dut = dut_source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, dut_source));
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake

/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <exception>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

GTEST_TEST(BulbColorTest, InstantiateAndAssign) {
  BulbColor dut{};
  EXPECT_EQ(dut, BulbColor::kRed);
  for (BulbColor color : {BulbColor::kGreen, BulbColor::kYellow}) {
    EXPECT_NE(dut, color);
    dut = color;
    EXPECT_EQ(dut, color);
  }
}

GTEST_TEST(BulbColorTest, MapperTest) {
  const auto dut = BulbColorMapper();
  const std::vector<BulbColor> expected_colors{
      BulbColor::kRed, BulbColor::kYellow, BulbColor::kGreen};
  EXPECT_EQ(dut.size(), expected_colors.size());
  for (BulbColor color : expected_colors) {
    EXPECT_EQ(dut.count(color), 1);
  }
}

GTEST_TEST(BulbTypeTest, InstantiateAndAssign) {
  BulbType dut{};
  EXPECT_EQ(dut, BulbType::kRound);
  for (BulbType type : {BulbType::kArrow}) {
    EXPECT_NE(dut, type);
    dut = type;
    EXPECT_EQ(dut, type);
  }
}

GTEST_TEST(BulbTypeTest, MapperTest) {
  const auto dut = BulbTypeMapper();
  const std::vector<BulbType> expected_types{BulbType::kRound,
                                             BulbType::kArrow};
  EXPECT_EQ(dut.size(), expected_types.size());
  for (BulbType type : expected_types) {
    EXPECT_EQ(dut.count(type), 1);
  }
}

GTEST_TEST(BulbStateTest, InstantiateAndAssign) {
  BulbState dut{};
  EXPECT_EQ(dut, BulbState::kOff);
  for (BulbState state : {BulbState::kOn, BulbState::kBlinking}) {
    EXPECT_NE(dut, state);
    dut = state;
    EXPECT_EQ(dut, state);
  }
}

GTEST_TEST(BulbStateTest, MapperTest) {
  const auto dut = BulbStateMapper();
  const std::vector<BulbState> expected_states{BulbState::kOff, BulbState::kOn,
                                               BulbState::kBlinking};
  EXPECT_EQ(dut.size(), expected_states.size());
  for (BulbState state : expected_states) {
    EXPECT_EQ(dut.count(state), 1);
  }
}

GTEST_TEST(BulbConstructorTest, ArrowWithoutOrientation) {
  EXPECT_THROW(
      Bulb(Bulb::Id("other_dut_id"), GeoPosition(7, 8, 9),
           Rotation::FromRpy(10, 11, 12), BulbColor::kGreen, BulbType::kArrow),
      std::exception);
}

GTEST_TEST(BulbConstructorTest, NonArrowWithOrientation) {
  EXPECT_THROW(Bulb(Bulb::Id("other_dut_id"), GeoPosition(7, 8, 9),
                    Rotation::FromRpy(10, 11, 12), BulbColor::kGreen,
                    BulbType::kRound, 0 /* arrow_orientation_rad */),
               std::exception);
}

class BulbTest : public ::testing::Test {
 public:
  BulbTest()
      : bulb_(Bulb::Id("dut_id"), GeoPosition(1, 2, 3),
              Rotation::FromRpy(4, 5, 6), BulbColor::kRed, BulbType::kRound) {}
  const Bulb bulb_;
};

TEST_F(BulbTest, Accessors) {
  EXPECT_EQ(bulb_.id(), Bulb::Id("dut_id"));
  EXPECT_EQ(bulb_.position_bulb_group(), GeoPosition(1, 2, 3));
  EXPECT_EQ(bulb_.orientation_bulb_group().matrix(),
            Rotation::FromRpy(4, 5, 6).matrix());
  EXPECT_EQ(bulb_.color(), BulbColor::kRed);
  EXPECT_EQ(bulb_.type(), BulbType::kRound);
  EXPECT_EQ(bulb_.states().size(), 2);
  EXPECT_EQ(bulb_.states().at(0), BulbState::kOff);
  EXPECT_EQ(bulb_.states().at(1), BulbState::kOn);
}

TEST_F(BulbTest, Copying) {
  const Bulb dut(bulb_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb_));
}

TEST_F(BulbTest, Assignment) {
  Bulb dut(Bulb::Id("other_dut_id"), GeoPosition(7, 8, 9),
           Rotation::FromRpy(10, 11, 12), BulbColor::kGreen, BulbType::kArrow,
           0 /* arrow_orientation_rad */,
           std::vector<BulbState>{BulbState::kBlinking});
  dut = bulb_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb_));
}

class BulbGroupTest : public ::testing::Test {
 public:
  BulbGroupTest()
      : red_bulb_(Bulb::Id("red_bulb"), GeoPosition(0, 0, -0.25),
                  Rotation::FromRpy(0, 0, 0), BulbColor::kRed,
                  BulbType::kRound),
        yellow_bulb_(Bulb::Id("yellow_bulb"), GeoPosition(0, 0, 0),
                     Rotation::FromRpy(0, 0, 0), BulbColor::kYellow,
                     BulbType::kRound),
        green_bulb_(Bulb::Id("green_bulb"), GeoPosition(0, 0, 0.25),
                    Rotation::FromRpy(0, 0, 0), BulbColor::kGreen,
                    BulbType::kRound),
        bulb_group_(BulbGroup::Id("test_bulb_group"), GeoPosition(1, 2, 3),
                    Rotation::FromRpy(4, 5, 6),
                    {red_bulb_, yellow_bulb_, green_bulb_}) {}

  const Bulb red_bulb_;
  const Bulb yellow_bulb_;
  const Bulb green_bulb_;
  const BulbGroup bulb_group_;
};

TEST_F(BulbGroupTest, Accessors) {
  EXPECT_EQ(bulb_group_.id(), BulbGroup::Id("test_bulb_group"));
  EXPECT_EQ(bulb_group_.position_traffic_light(), GeoPosition(1, 2, 3));
  EXPECT_EQ(bulb_group_.orientation_traffic_light().matrix(),
            Rotation::FromRpy(4, 5, 6).matrix());
  EXPECT_EQ(bulb_group_.bulbs().size(), 3);
}

TEST_F(BulbGroupTest, Copying) {
  const BulbGroup dut(bulb_group_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb_group_));
}

TEST_F(BulbGroupTest, Assignment) {
  const Bulb red_arrow_bulb(Bulb::Id("red_arrow_bulb"), GeoPosition(1, 2, 3),
                            Rotation::FromRpy(4, 5, 6), BulbColor::kRed,
                            BulbType::kArrow, 0 /* arrow_orientation_rad */);
  const Bulb green_arrow_bulb(
      Bulb::Id("green_arrow_bulb"), GeoPosition(7, 8, 9),
      Rotation::FromRpy(10, 11, 12), BulbColor::kGreen, BulbType::kArrow,
      M_PI / 2. /* arrow_orientation_rad */);

  BulbGroup dut(BulbGroup::Id("other_dut_id"), GeoPosition(13, 14, 15),
                Rotation::FromRpy(17, 18, 19),
                {red_arrow_bulb, green_arrow_bulb});
  dut = bulb_group_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb_group_));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake

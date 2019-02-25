/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <exception>

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
  constexpr int kNumColors{3};
  EXPECT_EQ(dut.size(), kNumColors);
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
  constexpr int kNumTypes{2};
  EXPECT_EQ(dut.size(), kNumTypes);
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
}

TEST_F(BulbTest, Copying) {
  const Bulb dut(bulb_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb_));
}

TEST_F(BulbTest, Assignment) {
  Bulb dut(Bulb::Id("other_dut_id"), GeoPosition(7, 8, 9),
           Rotation::FromRpy(10, 11, 12), BulbColor::kGreen, BulbType::kArrow,
           0 /* arrow_orientation_rad */);
  dut = bulb_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb_));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake

/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

GTEST_TEST(BulbColorTest, InstantiateAndAssign) {
  BulbColor dut{};
  EXPECT_EQ(dut, BulbColor::kUnknown);
  for (BulbColor color :
       {BulbColor::kGreen, BulbColor::kYellow, BulbColor::kRed}) {
    EXPECT_NE(dut, color);
    dut = color;
    EXPECT_EQ(dut, color);
  }
}

GTEST_TEST(BulbColorTest, MapperTest) {
  const auto dut = BulbColorMapper();
  constexpr int kNumColors{4};
  EXPECT_EQ(dut.size(), kNumColors);
}

GTEST_TEST(BulbTypeTest, InstantiateAndAssign) {
  BulbType dut{};
  EXPECT_EQ(dut, BulbType::kUnknown);
  for (BulbType type : {BulbType::kRound, BulbType::kArrow}) {
    EXPECT_NE(dut, type);
    dut = type;
    EXPECT_EQ(dut, type);
  }
}

GTEST_TEST(BulbTypeTest, MapperTest) {
  const auto dut = BulbTypeMapper();
  constexpr int kNumTypes{3};
  EXPECT_EQ(dut.size(), kNumTypes);
}

struct BulbTest : public ::testing::Test {
 public:
  BulbTest()
      : bulb(Bulb::Id("dut_id"), GeoPosition(1, 2, 3),
             Rotation::FromRpy(4, 5, 6), BulbColor::kRed, BulbType::kRound) {}
  const Bulb bulb;
};

TEST_F(BulbTest, Accessors) {
  EXPECT_EQ(bulb.id(), Bulb::Id("dut_id"));
  EXPECT_EQ(bulb.position_bulb_group(), GeoPosition(1, 2, 3));
  EXPECT_EQ(bulb.orientation_bulb_group().matrix(),
            Rotation::FromRpy(4, 5, 6).matrix());
  EXPECT_EQ(bulb.color(), BulbColor::kRed);
  EXPECT_EQ(bulb.type(), BulbType::kRound);
}

TEST_F(BulbTest, Copying) {
  const Bulb dut(bulb);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb));
}

TEST_F(BulbTest, Assignment) {
  Bulb dut(Bulb::Id("other_dut_id"), GeoPosition(7, 8, 9),
           Rotation::FromRpy(10, 11, 12), BulbColor::kRed, BulbType::kArrow);
  dut = bulb;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, bulb));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake

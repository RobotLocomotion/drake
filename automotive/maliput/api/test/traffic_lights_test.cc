/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <algorithm>
#include <exception>
#include <string>
#include <unordered_map>
#include <utility>
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

GTEST_TEST(BulbConstructorTest, EmptyAndNullOptStateVector) {
  const std::vector<Bulb> test_cases = {
      Bulb(Bulb::Id("empty_state_vector"), GeoPosition(0, 0, 0),
           Rotation::FromRpy(0, 0, 0), BulbColor::kGreen,
           BulbType::kRound, nullopt /* arrow_orientation_rad */,
           {} /* states */),
      Bulb(Bulb::Id("nullopt_state_vector"), GeoPosition(0, 0, 0),
           Rotation::FromRpy(0, 0, 0), BulbColor::kGreen,
           BulbType::kRound, nullopt /* arrow_orientation_rad */,
           nullopt /* states */)};
  for (const auto& test_case : test_cases) {
    EXPECT_EQ(test_case.states().size(), 2);
    EXPECT_EQ(test_case.GetDefaultState(), BulbState::kOff);
    EXPECT_TRUE(test_case.IsValidState(BulbState::kOff));
    EXPECT_TRUE(test_case.IsValidState(BulbState::kOn));
    EXPECT_FALSE(test_case.IsValidState(BulbState::kBlinking));
  }
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
  EXPECT_EQ(bulb_.GetDefaultState(), BulbState::kOff);
  EXPECT_TRUE(bulb_.IsValidState(BulbState::kOff));
  EXPECT_TRUE(bulb_.IsValidState(BulbState::kOn));
  EXPECT_FALSE(bulb_.IsValidState(BulbState::kBlinking));
  MALIPUT_IS_EQUAL(bulb_.bounding_box(), Bulb::BoundingBox());
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

GTEST_TEST(DefaultBulbStateTest, CorrectDefaultAndIsValidStateQueries) {
  struct TestCase {
    std::vector<BulbState> states;
    BulbState default_state;
  };
  const std::vector<TestCase> test_cases = {
      {{BulbState::kBlinking, BulbState::kOn}, BulbState::kBlinking},
      {{BulbState::kOn, BulbState::kBlinking}, BulbState::kBlinking},
      {{BulbState::kBlinking, BulbState::kOff}, BulbState::kOff},
      {{BulbState::kOff, BulbState::kBlinking}, BulbState::kOff},
      {{BulbState::kOff, BulbState::kOn}, BulbState::kOff},
      {{BulbState::kOn, BulbState::kOff}, BulbState::kOff},
      {{BulbState::kOn, BulbState::kOff, BulbState::kBlinking},
       BulbState::kOff},
      {{BulbState::kOff, BulbState::kOn, BulbState::kBlinking},
       BulbState::kOff},
      {{BulbState::kOff, BulbState::kBlinking, BulbState::kOn},
       BulbState::kOff},
      {{BulbState::kBlinking, BulbState::kOn, BulbState::kOff},
       BulbState::kOff},
      {{BulbState::kBlinking, BulbState::kOff, BulbState::kOn},
       BulbState::kOff},
      {{BulbState::kBlinking}, BulbState::kBlinking},
      {{BulbState::kOn}, BulbState::kOn},
      {{BulbState::kOff}, BulbState::kOff}};
  for (const auto& test_case : test_cases) {
    const Bulb dut(Bulb::Id("id"), GeoPosition(0, 0, 0),
                   Rotation::FromRpy(0, 0, 0), BulbColor::kGreen,
                   BulbType::kRound, nullopt /* arrow_orientation_rad */,
                   test_case.states);
    EXPECT_EQ(dut.GetDefaultState(), test_case.default_state);
    for (const auto& state : test_case.states) {
      EXPECT_TRUE(dut.IsValidState(state));
    }
  }
}

GTEST_TEST(BulbGroupConstructorTest, InvalidGroupSize) {
  EXPECT_THROW(BulbGroup(BulbGroup::Id("dut_id"), GeoPosition(1, 2, 3),
                         Rotation::FromRpy(M_PI, M_PI, M_PI), {}),
               std::exception);
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
  EXPECT_EQ(bulb_group_.GetBulb(Bulb::Id("unknown_bulb")), nullopt);
  EXPECT_NE(bulb_group_.GetBulb(Bulb::Id("red_bulb")), nullopt);
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

class TrafficLightTest : public ::testing::Test {
 public:
  TrafficLightTest()
      : north_bulb_(Bulb::Id("north_bulb"), GeoPosition(0, 0, 0),
                    Rotation::FromRpy(0, 0, 0), BulbColor::kRed,
                    BulbType::kRound),
        south_bulb_(Bulb::Id("south_bulb"), GeoPosition(0, 0, 0),
                    Rotation::FromRpy(0, 0, 0), BulbColor::kRed,
                    BulbType::kRound),
        east_bulb_(Bulb::Id("east_bulb"), GeoPosition(0, 0, 0),
                   Rotation::FromRpy(0, 0, 0), BulbColor::kRed,
                   BulbType::kRound),
        west_bulb_(Bulb::Id("west_bulb"), GeoPosition(0, 0, 0),
                   Rotation::FromRpy(0, 0, 0), BulbColor::kRed,
                   BulbType::kRound),
        north_bulb_group_(BulbGroup::Id("north_group"), GeoPosition(0, 0.1, 0),
                          Rotation::FromRpy(0, 0, M_PI_2), {north_bulb_}),
        south_bulb_group_(BulbGroup::Id("south_group"), GeoPosition(0, -0.1, 0),
                          Rotation::FromRpy(0, 0, -M_PI_2), {south_bulb_}),
        east_bulb_group_(BulbGroup::Id("east_group"), GeoPosition(0.1, 0, 0),
                         Rotation::FromRpy(0, 0, 0), {east_bulb_}),
        west_bulb_group_(BulbGroup::Id("west_group"), GeoPosition(-0.1, 0, 0),
                         Rotation::FromRpy(0, 0, M_PI), {west_bulb_}),
        traffic_light_(TrafficLight::Id("four_way_stop"), GeoPosition(0, 0, 5),
                       Rotation::FromRpy(0, 0, 0),
                       {north_bulb_group_, south_bulb_group_, east_bulb_group_,
                        west_bulb_group_}) {}

  const Bulb north_bulb_;
  const Bulb south_bulb_;
  const Bulb east_bulb_;
  const Bulb west_bulb_;

  const BulbGroup north_bulb_group_;
  const BulbGroup south_bulb_group_;
  const BulbGroup east_bulb_group_;
  const BulbGroup west_bulb_group_;

  const TrafficLight traffic_light_;
};

TEST_F(TrafficLightTest, Accessors) {
  EXPECT_EQ(traffic_light_.id(), TrafficLight::Id("four_way_stop"));
  EXPECT_EQ(traffic_light_.position_road_network(), GeoPosition(0, 0, 5));
  EXPECT_EQ(traffic_light_.orientation_road_network().matrix(),
            Rotation::FromRpy(0, 0, 0).matrix());
  EXPECT_EQ(traffic_light_.bulb_groups().size(), 4);
  EXPECT_EQ(traffic_light_.GetBulbGroup(BulbGroup::Id("unknown_bulb_group")),
            nullopt);
  EXPECT_NE(traffic_light_.GetBulbGroup(BulbGroup::Id("north_group")), nullopt);
}

TEST_F(TrafficLightTest, Copying) {
  const TrafficLight dut(traffic_light_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, traffic_light_));
}

TEST_F(TrafficLightTest, Assignment) {
  const Bulb green_arrow_bulb(
      Bulb::Id("green_arrow_bulb"), GeoPosition(-1, -2, -3),
      Rotation::FromRpy(-4, -5, -6), BulbColor::kGreen, BulbType::kArrow,
      M_PI_2 /* arrow_orientation_rad */);

  const BulbGroup bulb_group(BulbGroup::Id("other_bulb_group"),
                             GeoPosition(13, 14, 15),
                             Rotation::FromRpy(17, 18, 19), {green_arrow_bulb});
  TrafficLight dut(TrafficLight::Id("other_traffic_light"),
                   GeoPosition(10, 11, 12), Rotation::FromRpy(1, 2, 3),
                   {bulb_group});
  dut = traffic_light_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, traffic_light_));
}

GTEST_TEST(UniqueBulbIdTest, Usage) {
  const std::string traffic_light_name{"MyTrafficLight"};
  const std::string bulb_group_name{"MyBulbGroup"};
  const std::string bulb_name{"MyBulb"};

  const TrafficLight::Id traffic_light_id(traffic_light_name);
  const BulbGroup::Id bulb_group_id(bulb_group_name);
  const Bulb::Id bulb_id(bulb_name);

  const UniqueBulbId dut{traffic_light_id, bulb_group_id, bulb_id};

  // A mismatch of just one internal ID results in the UniqueBulbId no longer
  // matching.
  EXPECT_NE(dut,
            (UniqueBulbId{TrafficLight::Id("foo"), bulb_group_id, bulb_id}));
  EXPECT_NE(dut,
            (UniqueBulbId{traffic_light_id, BulbGroup::Id("foo"), bulb_id}));
  EXPECT_NE(dut,
            (UniqueBulbId{traffic_light_id, bulb_group_id, Bulb::Id("foo")}));

  const std::string dut_string = dut.to_string();
  for (const auto& name : {traffic_light_name, bulb_group_name, bulb_name}) {
    EXPECT_NE(dut_string.find(name), std::string::npos);
  }

  const UniqueBulbId copied_dut = dut;
  EXPECT_EQ(copied_dut, dut);

  UniqueBulbId assigned_dut{TrafficLight::Id("foo"), BulbGroup::Id("bar"),
                            Bulb::Id("baz")};
  EXPECT_NE(assigned_dut, dut);
  assigned_dut = dut;
  EXPECT_EQ(assigned_dut, dut);

  std::unordered_map<UniqueBulbId, BulbState> unordered_map;
  const BulbState bulb_state = BulbState::kOn;
  unordered_map.emplace(std::make_pair(dut, bulb_state));
  EXPECT_NE(unordered_map.find(dut), unordered_map.end());
  EXPECT_EQ(unordered_map.at(dut), bulb_state);
  const UniqueBulbId other_dut{TrafficLight::Id("foo"), BulbGroup::Id("bar"),
                               Bulb::Id("baz")};
  EXPECT_EQ(unordered_map.find(other_dut), unordered_map.end());

  std::map<UniqueBulbId, BulbState> ordered_map;
  ordered_map.emplace(std::make_pair(dut, bulb_state));
  EXPECT_NE(ordered_map.find(dut), ordered_map.end());
  EXPECT_EQ(ordered_map.at(dut), bulb_state);
  EXPECT_EQ(ordered_map.find(other_dut), ordered_map.end());

  // Tests the std::less<UniqueBulbId>() operator.
  auto make_less = [](const std::string& s) -> std::string {
    std::string c = s;
    // Letter "L" is considered less than letter  "M" because its ASCII code is
    // less (0x4C vs. 0x4D).
    std::replace(c.begin(), c.end(), 'M', 'L');
    return c;
  };
  auto make_more = [](const std::string& s) -> std::string {
    std::string c = s;
    // Letter "N" is greater than "M" because its ASCII code is higher (0x4E vs.
    // 0x4D).
    std::replace(c.begin(), c.end(), 'M', 'N');
    return c;
  };
  const TrafficLight::Id less_traffic_light_id(make_less(traffic_light_name));
  const TrafficLight::Id more_traffic_light_id(make_more(traffic_light_name));
  const BulbGroup::Id less_bulb_group_id(make_less(bulb_group_name));
  const BulbGroup::Id more_bulb_group_id(make_more(bulb_group_name));
  const Bulb::Id less_bulb_id(make_less(bulb_name));
  const Bulb::Id more_bulb_id(make_more(bulb_name));

  const std::less<UniqueBulbId> less;

  const std::vector<UniqueBulbId> less_set = {
      {less_traffic_light_id, bulb_group_id, bulb_id},
      {less_traffic_light_id, less_bulb_group_id, more_bulb_id},
      {less_traffic_light_id, more_bulb_group_id, more_bulb_id},
      {less_traffic_light_id, more_bulb_group_id, less_bulb_id},
      {traffic_light_id, less_bulb_group_id, bulb_id},
      {traffic_light_id, less_bulb_group_id, more_bulb_id},
      {traffic_light_id, bulb_group_id, less_bulb_id}};

  for (const auto& test_case : less_set) {
    EXPECT_TRUE(less(test_case, dut));
  }

  const std::vector<UniqueBulbId> not_less_set = {
      dut,
      {more_traffic_light_id, less_bulb_group_id, less_bulb_id},
      {more_traffic_light_id, more_bulb_group_id, less_bulb_id},
      {more_traffic_light_id, less_bulb_group_id, more_bulb_id},
      {more_traffic_light_id, more_bulb_group_id, more_bulb_id},
      {traffic_light_id, more_bulb_group_id, more_bulb_id},
      {traffic_light_id, more_bulb_group_id, less_bulb_id},
      {traffic_light_id, bulb_group_id, more_bulb_id}};

  for (const auto& test_case : not_less_set) {
    EXPECT_FALSE(less(test_case, dut));
  }
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake

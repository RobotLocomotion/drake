#include "drake/automotive/maliput/base/traffic_light_book_loader.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/traffic_light_book.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace {

using api::GeoPosition;
using api::Rotation;
using api::rules::Bulb;
using api::rules::BulbColor;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::BulbType;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;

class TestLoading2x2IntersectionTrafficLightbook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionTrafficLightbook()
      : filepath_(FindResourceOrThrow(
            "drake/automotive/maliput/multilane/2x2_intersection.yaml")),
        south_facing_(
            TrafficLight::Id("SouthFacing"), GeoPosition(1.875, 9.375, 6.0579),
            Rotation::FromQuat(Quaternion<double>(0.707107, 0, 0, -0.707107)),
            {BulbGroup(
                BulbGroup::Id("SouthFacingBulbs"), GeoPosition(0, 0, 0),
                Rotation::FromQuat(Quaternion<double>(1, 0, 0, 0)),
                {Bulb(Bulb::Id("RedBulb"), GeoPosition(0, 0, 0.3937),
                      Rotation::FromQuat(Quaternion<double>(1, 0, 0, 0)),
                      BulbColor::kRed, BulbType::kRound, nullopt,
                      std::vector<BulbState>({BulbState::kOn, BulbState::kOff}),
                      Bulb::BoundingBox()),
                 Bulb(Bulb::Id("YellowBulb"), GeoPosition(0, 0, 0),
                      Rotation::FromQuat(Quaternion<double>(1, 0, 0, 0)),
                      BulbColor::kYellow, BulbType::kRound, nullopt,
                      std::vector<BulbState>({BulbState::kOn, BulbState::kOff}),
                      Bulb::BoundingBox()),
                 Bulb(Bulb::Id("GreenBulb"), GeoPosition(0, 0, -0.3937),
                      Rotation::FromQuat(Quaternion<double>(1, 0, 0, 0)),
                      BulbColor::kGreen, BulbType::kRound, nullopt,
                      std::vector<BulbState>({BulbState::kOn, BulbState::kOff}),
                      Bulb::BoundingBox()),
                 Bulb(Bulb::Id("YellowLeftArrowBulb"),
                      GeoPosition(0, -0.3937, -0.3937),
                      Rotation::FromQuat(Quaternion<double>(1, 0, 0, 0)),
                      BulbColor::kYellow, BulbType::kArrow, 3.14,
                      std::vector<BulbState>(
                          {BulbState::kOff, BulbState::kBlinking}),
                      Bulb::BoundingBox())})}) {}

  const std::string filepath_;
  const TrafficLight south_facing_;
};

// Fully check the south-facing traffic light. Then spot-check the rest of them.
TEST_F(TestLoading2x2IntersectionTrafficLightbook, LoadFromFile) {
  std::unique_ptr<TrafficLightBook> book =
      LoadTrafficLightBookFromFile(filepath_);
  EXPECT_NE(book, nullptr);
  optional<TrafficLight> south_facing =
      book->GetTrafficLight(TrafficLight::Id("SouthFacing"));
  EXPECT_NE(south_facing, nullopt);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(*south_facing, south_facing_));
  for (const auto& name : {"NorthFacing", "EastFacing", "WestFacing"}) {
    optional<TrafficLight> traffic_light =
        book->GetTrafficLight(TrafficLight::Id(name));
    EXPECT_NE(traffic_light, nullopt);
  }
}

}  // namespace
}  // namespace maliput
}  // namespace drake

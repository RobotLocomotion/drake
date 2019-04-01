#include "drake/automotive/maliput/base/traffic_light_book.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::TrafficLight;

GTEST_TEST(TrafficLightBookTest, BasicTest) {
  const TrafficLight::Id id("my traffic light");
  const TrafficLight traffic_light(id, api::GeoPosition(10, 11, 12),
                                   api::Rotation::FromRpy(1, 2, 3), {});
  TrafficLightBook dut;
  dut.AddTrafficLight(traffic_light);
  EXPECT_EQ(dut.GetTrafficLight(TrafficLight::Id("unknown_traffic light")),
            nullopt);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(*dut.GetTrafficLight(id), traffic_light));
}

}  // namespace
}  // namespace maliput
}  // namespace drake

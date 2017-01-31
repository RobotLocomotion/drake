#include "drake/automotive/gen/endless_road_car_state_translator.h"

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "drake/automotive/gen/endless_road_car_state.h"

namespace drake {
namespace automotive {
namespace {

GTEST_TEST(EndlessRoadCarStateTranslatorTest, RoundtripTest) {
  // The device under test.
  EndlessRoadCarStateTranslator dut;

  // Sample data.
  EndlessRoadCarState<double> publish_state_vector;
  publish_state_vector.set_s(11.0);
  publish_state_vector.set_r(22.0);
  publish_state_vector.set_heading(33.0);
  publish_state_vector.set_speed(44.0);

  // Encode the message.
  double time = 0;
  std::vector<uint8_t> lcm_message_bytes;
  dut.Serialize(time, publish_state_vector, &lcm_message_bytes);
  EXPECT_GT(lcm_message_bytes.size(), 0u);

  // Decode the message.
  EndlessRoadCarState<double> subscribe_state_vector;
  dut.Deserialize(
      lcm_message_bytes.data(), lcm_message_bytes.size(),
      &subscribe_state_vector);

  // Data should match.
  EXPECT_EQ(11.0, subscribe_state_vector.s());
  EXPECT_EQ(22.0, subscribe_state_vector.r());
  EXPECT_EQ(33.0, subscribe_state_vector.heading());
  EXPECT_EQ(44.0, subscribe_state_vector.speed());

  // Output class is as expected.
  std::unique_ptr<systems::VectorBase<double>> allocated_vector =
      dut.AllocateOutputVector();
  ASSERT_NE(nullptr, allocated_vector.get());
  ASSERT_NE(nullptr, dynamic_cast<EndlessRoadCarState<double>*>(
      allocated_vector.get()));
}

}  // namespace
}  // namespace automotive
}  // namespace drake

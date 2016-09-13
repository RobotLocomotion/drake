#include "drake/automotive/gen/simple_car_state_translator.h"

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "drake/automotive/gen/simple_car_state.h"

namespace drake {
namespace cars {
namespace {

GTEST_TEST(SimpleCarStateTranslatorTest, RoundtripTest) {
  // The device under test.
  SimpleCarStateTranslator dut;

  // Sample data.
  SimpleCarState<double> publish_state_vector;
  publish_state_vector.set_x(11.0);
  publish_state_vector.set_y(22.0);
  publish_state_vector.set_heading(33.0);
  publish_state_vector.set_velocity(44.0);

  // Encode the message.
  std::vector<uint8_t> lcm_message_bytes;
  dut.TranslateVectorBaseToLcm(publish_state_vector, &lcm_message_bytes);
  EXPECT_GT(lcm_message_bytes.size(), 0);

  // Decode the message.
  SimpleCarState<double> subscribe_state_vector;
  dut.TranslateLcmToVectorBase(
      lcm_message_bytes.data(), lcm_message_bytes.size(),
      &subscribe_state_vector);

  // Data should match.
  EXPECT_EQ(11.0, subscribe_state_vector.x());
  EXPECT_EQ(22.0, subscribe_state_vector.y());
  EXPECT_EQ(33.0, subscribe_state_vector.heading());
  EXPECT_EQ(44.0, subscribe_state_vector.velocity());

  // Output class is as expected.
  std::unique_ptr<systems::VectorBase<double>> allocated_vector =
      dut.AllocateOutputVector();
  ASSERT_NE(nullptr, allocated_vector.get());
  ASSERT_NE(nullptr, dynamic_cast<SimpleCarState<double>*>(
      allocated_vector.get()));
}

}  // namespace
}  // namespace cars
}  // namespace drake

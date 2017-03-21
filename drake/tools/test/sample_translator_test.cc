#include "drake/tools/test/gen/sample_translator.h"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "drake/tools/test/gen/sample.h"

namespace drake {
namespace tools {
namespace test {
namespace {

GTEST_TEST(SampleTranslatorTest, RoundtripTest) {
  // The device under test.
  SampleTranslator dut;

  // Sample data.
  Sample<double> publish_vector;
  publish_vector.set_x(11.0);
  publish_vector.set_two_word(22.0);

  // Encode the message.
  double time = 0;
  std::vector<uint8_t> lcm_message_bytes;
  dut.Serialize(time, publish_vector, &lcm_message_bytes);
  EXPECT_GT(lcm_message_bytes.size(), 0u);

  // Decode the message.
  Sample<double> subscribe_vector;
  dut.Deserialize(
      lcm_message_bytes.data(), lcm_message_bytes.size(),
      &subscribe_vector);

  // Data should match.
  EXPECT_EQ(11.0, subscribe_vector.x());
  EXPECT_EQ(22.0, subscribe_vector.two_word());

  // Output class is as expected.
  std::unique_ptr<systems::VectorBase<double>> allocated_vector =
      dut.AllocateOutputVector();
  ASSERT_NE(nullptr, allocated_vector.get());
  ASSERT_NE(nullptr, dynamic_cast<Sample<double>*>(
      allocated_vector.get()));
}

}  // namespace
}  // namespace test
}  // namespace tools
}  // namespace drake

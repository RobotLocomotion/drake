#include "drake/systems/lcm/serializer.h"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/is_cloneable.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace systems {
namespace lcm {
namespace {

using drake::lcm::CompareLcmtDrakeSignalMessages;

GTEST_TEST(SerializerTest, BasicTest) {
  const std::string channel_name{"channel_name"};

  // The device under test.
  auto dut = std::make_unique<Serializer<lcmt_drake_signal>>();

  // The default value should be zeroed.
  auto abstract_value = dut->CreateDefaultValue();
  const auto& value = abstract_value->get_value<lcmt_drake_signal>();
  EXPECT_EQ(value.dim, 0);
  EXPECT_EQ(value.val.size(), 0u);
  EXPECT_EQ(value.coord.size(), 0u);
  EXPECT_EQ(value.timestamp, 0);

  // Sample data should round-trip successfully.
  const lcmt_drake_signal sample_data{
    2,
    { 1.0, 2.0, },
    { "x", "y", },
    12345,
  };
  std::vector<uint8_t> message_bytes;
  dut->Serialize(Value<lcmt_drake_signal>(sample_data), &message_bytes);
  dut->Deserialize(message_bytes.data(), message_bytes.size(),
                   abstract_value.get());
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(
      abstract_value->get_value<lcmt_drake_signal>(), sample_data));

  // Cloning works.
  EXPECT_TRUE(is_cloneable<SerializerInterface>::value);
  auto fresh = dut->Clone();
  ASSERT_NE(fresh, nullptr);
  auto fresh_value = fresh->CreateDefaultValue();
  EXPECT_EQ(fresh_value->get_value<lcmt_drake_signal>().dim, 0);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

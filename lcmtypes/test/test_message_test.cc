#include "drake/test_message.hpp"

#include <gtest/gtest.h>

#include "drake/lcm/lcm_messages.h"

namespace drake {
namespace lcm {
namespace {

GTEST_TEST(LcmTestMessageTest, Smoke) {
  test_message foo{.value = 22.0};
  const std::vector<uint8_t> bytes = EncodeLcmMessage(foo);
  auto readback = DecodeLcmMessage<test_message>(bytes);
  EXPECT_EQ(readback.value, foo.value);
}

}  // namespace
}  // namespace lcm
}  // namespace drake

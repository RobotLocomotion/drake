#include "drake/lcm/lcm_messages.h"

#include <gtest/gtest.h>

#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

lcmt_drake_signal ModelMessage() {
  lcmt_drake_signal message{};
  message.timestamp = 2;
  return message;
}

GTEST_TEST(LcmMessagesTest, EncodeDecodeLcmMessage) {
  lcmt_drake_signal message{};
  EXPECT_FALSE(
      CompareLcmtDrakeSignalMessages(message, ModelMessage()));
  std::vector<uint8_t> bytes = EncodeLcmMessage(ModelMessage());
  EXPECT_GT(bytes.size(), 0);
  message = DecodeLcmMessage<lcmt_drake_signal>(bytes);
  EXPECT_TRUE(
      CompareLcmtDrakeSignalMessages(message, ModelMessage()));
}

GTEST_TEST(LcmMessagesTest, AreLcmMessagesEqual) {
  lcmt_drake_signal message{};
  EXPECT_FALSE(AreLcmMessagesEqual(message, ModelMessage()));
  EXPECT_TRUE(AreLcmMessagesEqual(ModelMessage(), ModelMessage()));
}

}  // namespace
}  // namespace lcm
}  // namespace drake

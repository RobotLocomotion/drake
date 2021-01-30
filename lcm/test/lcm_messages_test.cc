#include "drake/lcm/lcm_messages.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
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
  const std::vector<uint8_t> bytes = EncodeLcmMessage(ModelMessage());
  EXPECT_GT(bytes.size(), 0);
  message = DecodeLcmMessage<lcmt_drake_signal>(bytes);
  EXPECT_TRUE(
      CompareLcmtDrakeSignalMessages(message, ModelMessage()));
}

GTEST_TEST(LcmMessagesTest, DecodeLcmMessageError) {
  const std::vector<uint8_t> bytes = EncodeLcmMessage(ModelMessage());
  std::vector<uint8_t> corrupt_bytes = bytes;
  corrupt_bytes.at(0) = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      DecodeLcmMessage<lcmt_drake_signal>(corrupt_bytes),
      std::runtime_error,
      "Error decoding message of type 'drake::lcmt_drake_signal'");
}

GTEST_TEST(LcmMessagesTest, AreLcmMessagesEqual) {
  lcmt_drake_signal message{};
  EXPECT_FALSE(AreLcmMessagesEqual(message, ModelMessage()));
  EXPECT_TRUE(AreLcmMessagesEqual(ModelMessage(), ModelMessage()));
}

}  // namespace
}  // namespace lcm
}  // namespace drake

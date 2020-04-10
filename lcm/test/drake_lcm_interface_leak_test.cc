#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

// This file holds test cases that we need to disable the leak checkers on.
// These tests are semantically part of the drake_lcm_interface_test.cc.

namespace drake {
namespace lcm {
namespace {

// Tests the Subscribe free function's default error handling.
GTEST_TEST(DrakeLcmInterfaceLeakTest, DefaultErrorHandlingTest) {
  DrakeLcm lcm;
  const std::string channel = "NAME";

  // A convenient, populated sample message and its encoded form.
  using Message = lcmt_drake_signal;
  Message sample{};
  sample.timestamp = 123;
  std::vector<uint8_t> sample_bytes;
  const int num_bytes = sample.getEncodedSize();
  DRAKE_DEMAND(num_bytes >= 0);
  sample_bytes.resize(static_cast<size_t>(num_bytes));
  sample.encode(sample_bytes.data(), 0, num_bytes);

  // Subscribe using the helper free-function, using default error-handling.
  Message received{};
  Subscribe<Message>(&lcm, channel, [&](const Message& message) {
    received = message;
  });

  // Publish successfully.
  lcm.Publish(channel, sample_bytes.data(), sample_bytes.size(), {});
  EXPECT_EQ(lcm.HandleSubscriptions(0), 1);
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, sample));
  received = {};

  // Corrupt the message.  The user should receive an exception.  Note that the
  // exception passes ACROSS THE C CODE in liblcm so the "free the packet we
  // just received" code is skipped, so it will show up as leaked memory.
  std::vector<uint8_t> corrupt_bytes = sample_bytes;
  corrupt_bytes.at(0) = 0;
  lcm.Publish(channel, corrupt_bytes.data(), corrupt_bytes.size(), {});
  DRAKE_EXPECT_THROWS_MESSAGE(
      lcm.HandleSubscriptions(0),
      std::runtime_error,
      "Error decoding message on NAME");
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, Message{}));
}

}  // namespace
}  // namespace lcm
}  // namespace drake

#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_drake_signal.hpp"

using std::string;
using std::vector;

namespace drake {
namespace lcm {
namespace {

// Handles received LCM messages.
class MockMessageHandler final {
 public:
  void Subscribe(const std::string& channel, DrakeLcmInterface* dut) {
    auto subscription = dut->Subscribe(
        channel, [this](const void* data, int size) {
          this->HandleMessage(data, size);
        });
    // Can't unsubscribe.
    DRAKE_DEMAND(subscription == nullptr);
  }

  // This is the callback method.
  void HandleMessage(const void* message_buffer, int message_size) {
    buffer_.resize(message_size);
    if (message_size > 0) {
      std::memcpy(&buffer_[0], message_buffer, message_size);
    }
  }

  vector<uint8_t>& get_buffer() {
    return buffer_;
  }

 private:
  vector<uint8_t> buffer_;
};

// Tests that DrakeMockLcm publishes and subscribes properly.
GTEST_TEST(DrakeMockLcmTest, AcceptanceTest) {
  const string kChannelName =
      "drake_mock_lcm_test_without_loopback_channel";

  // Instantiates the Device Under Test (DUT). Note that loopback is by default
  // disabled, which results in subscribers not being notified when a message is
  // published on the subscribed channel.
  DrakeMockLcm dut;

  // Instantiates a message handler.
  MockMessageHandler handler;
  handler.Subscribe(kChannelName, &dut);

  // Defines a fake serialized message.
  const int kMessageSize = 10;
  vector<uint8_t> message_bytes(kMessageSize);
  for (int i = 0; i < kMessageSize; ++i) {
    message_bytes[i] = i;
  }
  dut.Publish(kChannelName, &message_bytes[0], kMessageSize, nullopt);

  // Verifies that the message was not yet received.
  EXPECT_EQ(handler.get_buffer().size(), 0);

  // Calling Handle pushes the message.
  dut.HandleSubscriptions(0);
  EXPECT_EQ(handler.get_buffer().size(), kMessageSize);

  // A second Handle does nothing (it was already handled).
  handler.get_buffer().clear();
  dut.HandleSubscriptions(0);
  EXPECT_EQ(handler.get_buffer().size(), 0);
}

GTEST_TEST(DrakeMockLcmTest, EmptyChannelTest) {
  DrakeMockLcm dut;

  MockMessageHandler handler;
  EXPECT_THROW(handler.Subscribe("", &dut), std::exception);

  lcmt_drake_signal message{};
  EXPECT_THROW(Publish(&dut, "", message), std::exception);
}

}  // namespace
}  // namespace lcm
}  // namespace drake

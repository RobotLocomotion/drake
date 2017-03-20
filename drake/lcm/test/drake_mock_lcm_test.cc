#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcmt_drake_signal.hpp"

using std::string;
using std::vector;

namespace drake {
namespace lcm {
namespace {

// Tests DrakeMockLcm's ability to "publish" an LCM message.
GTEST_TEST(DrakeMockLcmTest, PublishTest) {
  const string channel_name = "drake_mock_lcm_test_publisher_channel_name";
  const int message_size = 10;
  vector<uint8_t> message_bytes(message_size);

  for (int i = 0; i < message_size; ++i) {
    message_bytes[i] = i;
  }

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;

  dut.StartReceiveThread();
  dut.Publish(channel_name, &message_bytes[0], message_size);

  // Verifies that the message was "published".
  const vector<uint8_t>& published_message_bytes =
      dut.get_last_published_message(channel_name);

  EXPECT_EQ(message_size, static_cast<int>(published_message_bytes.size()));
  EXPECT_EQ(message_bytes, published_message_bytes);
}

// Tests DrakeMockLcm::DecodeLastPublishedMessageAs() using an lcmt_drake_signal
// message.
GTEST_TEST(DrakeMockLcmTest, DecodeLastPublishedMessageAsTest) {
  const string channel_name = "DecodeLastPublishedMessageAsTestChannel";
  lcmt_drake_signal original_message;
  original_message.dim = 1;
  original_message.val.push_back(3.14);
  original_message.coord.push_back("foo");
  original_message.timestamp = 0;

  const int message_size = original_message.getEncodedSize();

  vector<uint8_t> message_bytes;
  message_bytes.resize(message_size);
  original_message.encode(message_bytes.data(), 0, message_bytes.size());

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;
  dut.StartReceiveThread();
  dut.Publish(channel_name, &message_bytes[0], message_size);

  // Verifies that the message was "published".
  const lcmt_drake_signal last_published_message =
      dut.DecodeLastPublishedMessageAs<lcmt_drake_signal>(channel_name);

  EXPECT_EQ(last_published_message.dim, original_message.dim);
  EXPECT_EQ(last_published_message.val.at(0), original_message.val.at(0));
  EXPECT_EQ(last_published_message.coord.at(0), original_message.coord.at(0));
  EXPECT_EQ(last_published_message.timestamp, original_message.timestamp);

  // Verifies that exceptions are thrown when a decode operation fails.
  dut.Publish(channel_name, &message_bytes[0], message_size - 1);
  EXPECT_THROW(dut.DecodeLastPublishedMessageAs<lcmt_drake_signal>(
      channel_name), std::runtime_error);

  message_bytes.push_back(0);
  dut.Publish(channel_name, &message_bytes[0], message_size + 1);
  EXPECT_THROW(dut.DecodeLastPublishedMessageAs<lcmt_drake_signal>(
      channel_name), std::runtime_error);
}

// Handles received LCM messages.
class MockMessageHandler : public DrakeLcmMessageHandlerInterface {
 public:
  // A constructor that initializes the memory for storing received LCM
  // messages.
  MockMessageHandler() { }

  // This is the callback method.
  void HandleMessage(const string& channel, const void* message_buffer,
      int message_size) override {
    channel_ = channel;
    buffer_.resize(message_size);
    std::memcpy(&buffer_[0], message_buffer, message_size);
  }

  const string& get_channel() const {
    return channel_;
  }

  const vector<uint8_t>& get_buffer() {
    return buffer_;
  }

  int get_buffer_size() {
    return buffer_.size();
  }

 private:
  string channel_{};
  vector<uint8_t> buffer_;
};

// Tests DrakeMockLcm's ability to "subscribe" to an LCM channel.
GTEST_TEST(DrakeMockLcmTest, SubscribeTest) {
  const string kChannelName =
      "drake_mock_lcm_test_subscriber_channel_name";

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;

  // Instantiates a message handler.
  MockMessageHandler handler;

  dut.StartReceiveThread();
  dut.Subscribe(kChannelName, &handler);

  // Defines a fake serialized message.
  const int kMessageSize = 10;
  vector<uint8_t> message_bytes(kMessageSize);

  for (int i = 0; i < kMessageSize; ++i) {
    message_bytes[i] = i;
  }

  dut.InduceSubscriberCallback(kChannelName, &message_bytes[0], kMessageSize);

  EXPECT_EQ(kChannelName, handler.get_channel());
  EXPECT_EQ(kMessageSize, handler.get_buffer_size());

  const vector<uint8_t>& received_bytes = handler.get_buffer();
  EXPECT_EQ(message_bytes, received_bytes);

  dut.StopReceiveThread();

  // Verifies that no more messages are received by subscribers after the
  // receive thread is stopped.
  vector<uint8_t> message_bytes2;
  message_bytes2.push_back(128);

  dut.InduceSubscriberCallback("foo_channel", &message_bytes2[0],
      message_bytes2.size());

  // Verifies that the original message is returned, not the one that was
  // sent after the receive thread was stopped.
  EXPECT_EQ(kChannelName, handler.get_channel());
  EXPECT_EQ(kMessageSize, handler.get_buffer_size());
  const vector<uint8_t>& received_bytes2 = handler.get_buffer();
  EXPECT_EQ(message_bytes, received_bytes2);
}

// Tests DrakeMockLcm's ability to "publish" and "subscribe" to an LCM channel
// via a loopback.
GTEST_TEST(DrakeMockLcmTest, WithLoopbackTest) {
  const string kChannelName =
      "drake_mock_lcm_test_loopback_channel";

  // Instantiates the Device Under Test (DUT). Note that loopback is enabled,
  // which results in subscribers being notified when a message is published on
  // the subscribed channel.
  DrakeMockLcm dut;
  dut.EnableLoopBack();

  // Instantiates a message handler.
  MockMessageHandler handler;

  dut.StartReceiveThread();
  dut.Subscribe(kChannelName, &handler);

  // Defines a fake serialized message.
  const int kMessageSize = 10;
  vector<uint8_t> message_bytes(kMessageSize);

  for (int i = 0; i < kMessageSize; ++i) {
    message_bytes[i] = i;
  }

  dut.Publish(kChannelName, &message_bytes[0], kMessageSize);

  // Verifies that the message was received via loopback.
  EXPECT_EQ(kChannelName, handler.get_channel());
  EXPECT_EQ(kMessageSize, handler.get_buffer_size());

  const vector<uint8_t>& received_bytes = handler.get_buffer();
  EXPECT_EQ(message_bytes, received_bytes);
}

// Tests that DrakeMockLcm will not loopback a message when loopback is
// disabled.
GTEST_TEST(DrakeMockLcmTest, WithoutLoopbackTest) {
  const string kChannelName =
      "drake_mock_lcm_test_without_loopback_channel";

  // Instantiates the Device Under Test (DUT). Note that loopback is by default
  // disabled, which results in subscribers not being notified when a message is
  // published on the subscribed channel.
  DrakeMockLcm dut;

  // Instantiates a message handler.
  MockMessageHandler handler;

  dut.StartReceiveThread();
  dut.Subscribe(kChannelName, &handler);

  // Defines a fake serialized message.
  const int kMessageSize = 10;
  vector<uint8_t> message_bytes(kMessageSize);

  for (int i = 0; i < kMessageSize; ++i) {
    message_bytes[i] = i;
  }

  dut.Publish(kChannelName, &message_bytes[0], kMessageSize);

  // Verifies that the message was not received via loopback.
  EXPECT_NE(kChannelName, handler.get_channel());
  EXPECT_NE(kMessageSize, handler.get_buffer_size());
}

}  // namespace
}  // namespace lcm
}  // namespace drake

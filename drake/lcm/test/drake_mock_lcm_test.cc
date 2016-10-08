#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>

#include "gtest/gtest.h"

#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {
namespace {

// Tests DrakeMockLcm's ability to "publish" an LCM message.
GTEST_TEST(DrakeMockLcmTest, PublishTest) {
  const std::string channel_name = "drake_mock_lcm_test_publisher_channel_name";
  const int message_size = 10;
  std::vector<uint8_t> message_bytes(message_size);

  for (int i = 0; i < message_size; ++i) {
    message_bytes[i] = i;
  }

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;

  dut.StartReceiveThread();
  dut.Publish(channel_name, &message_bytes[0], message_size);

  // Verifies that the message was "published".
  const std::vector<uint8_t>& published_message_bytes =
      dut.get_last_published_message(channel_name);

  EXPECT_EQ(message_size, published_message_bytes.size());
  EXPECT_EQ(message_bytes, published_message_bytes);
}

// Handles received LCM messages.
class MockMessageHandler : public DrakeLcmMessageHandlerInterface {
 public:
  // A constructor that initializes the memory for storing received LCM
  // messages.
  MockMessageHandler() { }

  // This is the callback method.
  void HandleMessage(const std::string& channel, const void* message_buffer,
      int message_size) override {
    channel_ = channel;
    buffer_.resize(message_size);
    std::memcpy(&buffer_[0], message_buffer, message_size);
  }

  const std::string& get_channel() const {
    return channel_;
  }

  const std::vector<uint8_t>& get_buffer() {
    return buffer_;
  }

  int get_buffer_size() {
    return buffer_.size();
  }

 private:
  std::string channel_{};
  std::vector<uint8_t> buffer_;
};

// Tests DrakeMockLcm's ability to "subscribe" to an LCM channel.
GTEST_TEST(DrakeMockLcmTest, SubscribeTest) {
  const std::string kChannelName =
      "drake_mock_lcm_test_subscriber_channel_name";

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;

  // Instantiates a message handler.
  MockMessageHandler handler;

  dut.StartReceiveThread();
  dut.Subscribe(kChannelName, &handler);

  // Defines a fake serialized message.
  const int kMessageSize = 10;
  std::vector<uint8_t> message_bytes(kMessageSize);

  for (int i = 0; i < kMessageSize; ++i) {
    message_bytes[i] = i;
  }

  dut.InduceSubscriberCallback(kChannelName, &message_bytes[0], kMessageSize);

  EXPECT_EQ(kChannelName, handler.get_channel());
  EXPECT_EQ(kMessageSize, handler.get_buffer_size());

  const std::vector<uint8_t>& received_bytes = handler.get_buffer();
  EXPECT_EQ(message_bytes, received_bytes);

  dut.StopReceiveThread();

  // Verifies that no more messages are received by subscribers after the
  // receive thread is stopped.
  std::vector<uint8_t> message_bytes2;
  message_bytes2.push_back(128);

  dut.InduceSubscriberCallback("foo_channel", &message_bytes2[0],
      message_bytes2.size());

  // Verifies that the original message is returned, not the one that was
  // sent after the receive thread was stopped.
  EXPECT_EQ(kChannelName, handler.get_channel());
  EXPECT_EQ(kMessageSize, handler.get_buffer_size());
  const std::vector<uint8_t>& received_bytes2 = handler.get_buffer();
  EXPECT_EQ(message_bytes, received_bytes2);
}

}  // namespace
}  // namespace lcm
}  // namespace drake

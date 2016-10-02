#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>

#include "gtest/gtest.h"

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {
namespace {

// Tests DrakeMockLcm's ability to "publish" an LCM message.
GTEST_TEST(DrakeMockLcmTest, PublishTest) {
  const std::string channel_name = "drake_mock_lcm_test_publisher_channel_name";
  const int message_size = 10;
  uint8_t message_bytes[message_size];

  for (int i = 0; i < message_size; ++i) {
    message_bytes[i] = i;
  }

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;

  dut.StartReceiveThread();
  dut.Publish(channel_name, message_bytes, message_size);

  // Verifies that the message was "published".
  std::string published_channel{};
  void* published_message_bytes = nullptr;
  unsigned int published_message_size{};

  EXPECT_TRUE(dut.get_last_published_message(&published_channel,
    &published_message_bytes, &published_message_size));

  EXPECT_EQ(published_channel, channel_name);
  EXPECT_EQ(message_size, published_message_size);
  for (int i = 0; i < message_size; ++i) {
    EXPECT_EQ(static_cast<uint8_t*>(published_message_bytes)[i],
        message_bytes[i]);
  }
}

// Handles received LCM messages.
class MockMessageHandler : public DrakeLcmMessageHandlerInterface {
 public:
  // A constructor that initializes the memory for storing received LCM
  // messages.
  MockMessageHandler() { }

  // This is the callback method.
  void HandleMessage(const std::string& channel, const void* message_buffer,
      uint32_t message_size) override {
    channel_ = channel;
    // std::lock_guard<std::mutex> lock(message_mutex_)
    buffer_.resize(message_size);
    std::memcpy(&buffer_[0], message_buffer, message_size);
  }

  const std::string& get_channel() const {
    return channel_;
  }

  uint8_t* get_buffer() {
    return &buffer_[0];
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
  const std::string channel_name =
      "drake_mock_lcm_test_subscriber_channel_name";

  // Instantiates the Device Under Test (DUT).
  DrakeMockLcm dut;

  // Instantiates a message handler.
  MockMessageHandler handler;

  dut.StartReceiveThread();
  dut.Subscribe(channel_name, &DrakeLcmMessageHandlerInterface::HandleMessage,
      &handler);

  // Defines a fake serialized message.
  const int message_size = 10;
  uint8_t message_bytes[message_size];

  for (int i = 0; i < message_size; ++i) {
    message_bytes[i] = i;
  }

  dut.InduceSubsciberCallback(channel_name, message_bytes, message_size);

  EXPECT_EQ(handler.get_buffer_size(), message_size);

  uint8_t* received_bytes = handler.get_buffer();
  for (int i = 0; i < message_size; ++i) {
    EXPECT_EQ(received_bytes[i], message_bytes[i]);
  }
}

}  // namespace
}  // namespace lcm
}  // namespace drake

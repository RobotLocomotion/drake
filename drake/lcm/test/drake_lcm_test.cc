#include "drake/lcm/drake_lcm.h"

#include <chrono>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcm/lcm_receive_thread.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// Subscribes to LCM messages of type `drake::lcmt_drake_signal`. Provides an
// accessor to the latest message received.
class MessageSubscriber {
 public:
  // A constructor that sets up the LCM message subscription and initializes the
  // member variable that will be used to store received LCM messages.
  MessageSubscriber(const std::string& channel_name, ::lcm::LCM* lcm)
      : channel_name_(channel_name) {
    ::lcm::Subscription* sub =
        lcm->subscribe(channel_name, &MessageSubscriber::HandleMessage, this);
    sub->setQueueCapacity(1);

    // Initializes the fields of received_message_ so the test logic below can
    // determine whether the desired message was received.
    received_message_.dim = 0;
    received_message_.val.resize(received_message_.dim);
    received_message_.coord.resize(received_message_.dim);
    received_message_.timestamp = 0;
  }

  // Returns a copy of the most recently received message.
  drake::lcmt_drake_signal GetReceivedMessage() {
    drake::lcmt_drake_signal message_copy;
    std::lock_guard<std::mutex> lock(message_mutex_);
    message_copy = received_message_;
    return message_copy;
  }

 private:
  // Saves a copy of the most recently received message.
  void HandleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel_name) {
    if (channel_name_ == channel_name) {
      std::lock_guard<std::mutex> lock(message_mutex_);
      // Note: The call to decode() below returns the number of bytes decoded
      // or a negative value indicating an error occurred. This error is
      // ignored since the unit test below includes logic that checks every
      // value within the received message for correctness.
      received_message_.decode(rbuf->data, 0, rbuf->data_size);
    }
  }

  const std::string channel_name_;
  std::mutex message_mutex_;
  drake::lcmt_drake_signal received_message_;
};

// This is a test fixture that defines a `drake::lcmt_drake_signal` message.
class DrakeLcmTest : public ::testing::Test {
 protected:
  void SetUp() override {
    message_.dim = 2;
    message_.val.push_back(0.3739558136);
    message_.val.push_back(0.2801694990);
    message_.coord.push_back("artin's constant");
    message_.coord.push_back("bernstein's constant");
    message_.timestamp = 142857;
  }

  drake::lcmt_drake_signal message_;
};

// Tests DrakeLcm's ability to publish an LCM message.
TEST_F(DrakeLcmTest, PublishTest) {
  const std::string channel_name = "drake_lcm_test_publisher_channel_name";

  // Instantiates the Device Under Test (DUT).
  DrakeLcm dut;

  MessageSubscriber subscriber(channel_name, dut.get_lcm_instance());

  std::vector<uint8_t> buffer(message_.getEncodedSize());
  EXPECT_EQ(message_.encode(&buffer[0], 0, message_.getEncodedSize()),
            message_.getEncodedSize());

  // Start the LCM recieve thread after all objects it can potentially use like
  // subscribers are instantiated. Since objects are destructed in the reverse
  // order of construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the Lcm receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  dut.StartReceiveThread();

  // Records whether the receiver received an LCM message published by the DUT.
  bool done = false;

  // Prevents this unit test from running indefinitely when the receiver fails
  // to receive the LCM message published by the DUT.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  // We must periodically call dut.Publish(...) since we do not know when the
  // receiver will actually receive the message.
  while (!done && count++ < kMaxCount) {
    dut.Publish(channel_name, &buffer[0], message_.getEncodedSize());

    // Gets the received message.
    const drake::lcmt_drake_signal received_message =
        subscriber.GetReceivedMessage();

    done = CompareLcmtDrakeSignalMessages(received_message, message_);

    if (!done) sleep_for(milliseconds(kDelayMS));
  }

  dut.StopReceiveThread();
  EXPECT_TRUE(done);
}

// Handles received LCM messages.
class MessageHandler : public DrakeLcmMessageHandlerInterface {
 public:
  // A constructor that initializes the memory for storing received LCM
  // messages.
  MessageHandler() {
    // Initializes the fields of received_message_ so the test logic can
    // determine whether the desired message was received.
    received_message_.dim = 0;
    received_message_.val.resize(received_message_.dim);
    received_message_.coord.resize(received_message_.dim);
    received_message_.timestamp = 0;
  }

  // This is the callback method.
  void HandleMessage(const std::string& channel, const void* message_buffer,
      int message_size) override {
    channel_ = channel;
    std::lock_guard<std::mutex> lock(message_mutex_);
    received_message_.decode(message_buffer, 0, message_size);
  }

  // Returns a copy of the most recently received message.
  drake::lcmt_drake_signal GetReceivedMessage() {
    drake::lcmt_drake_signal message_copy;
    std::lock_guard<std::mutex> lock(message_mutex_);
    message_copy = received_message_;
    return message_copy;
  }

  // Returns the channel on which the most recent message was received.
  const std::string& get_receive_channel() {
    return channel_;
  }

 private:
  std::string channel_{};
  std::mutex message_mutex_;
  drake::lcmt_drake_signal received_message_;
};

// Tests DrakeLcm's ability to subscribe to an LCM message.
TEST_F(DrakeLcmTest, SubscribeTest) {
  const std::string channel_name = "drake_lcm_subscriber_channel_name";

  // Instantiates the Device Under Test (DUT).
  DrakeLcm dut;

  MessageHandler handler;
  dut.Subscribe(channel_name, &handler);

  // Starts the LCM receive thread after the subscribers are created.
  dut.StartReceiveThread();
  ::lcm::LCM* lcm = dut.get_lcm_instance();

  // Prevents this unit test from running indefinitely when the receiver fails
  // to receive the LCM message published by the DUT.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  bool done = false;

  // We must periodically call dut.Publish(...) since we do not know when the
  // receiver will actually receive the message.
  while (!done && count++ < kMaxCount) {
    lcm->publish(channel_name, &message_);
    if (handler.get_receive_channel() == channel_name) {
      // Gets the received message.
      const drake::lcmt_drake_signal received_message =
          handler.GetReceivedMessage();
      done = CompareLcmtDrakeSignalMessages(received_message, message_);
    }
    if (!done) sleep_for(milliseconds(kDelayMS));
  }

  dut.StopReceiveThread();
  EXPECT_TRUE(done);
}

}  // namespace
}  // namespace lcm
}  // namespace drake

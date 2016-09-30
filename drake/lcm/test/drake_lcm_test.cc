#include "drake/lcm/drake_lcm.h"

// #include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"

// #include "drake/systems/framework/basic_vector.h"
// #include "drake/systems/lcm/lcm_receive_thread.h"
// #include "drake/systems/lcm/lcm_publisher_system.h"
// #include "drake/systems/lcm/lcm_translator_dictionary.h"
// #include "drake/systems/lcm/lcmt_drake_signal_translator.h"

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
  MessageSubscriber(const std::string& channel_name)
      : channel_name_(channel_name) {
    // Sets up the LCM message subscriber.
    ::lcm::Subscription* sub =
        lcm_.subscribe(channel_name, &MessageSubscriber::HandleMessage, this);
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

  ::lcm::LCM lcm_;
  const std::string channel_name_;
  std::mutex message_mutex_;
  drake::lcmt_drake_signal received_message_;
};

// Tests DrakeLcm's ability to publish an LCM message.
GTEST_TEST(DrakeLcmTest, PublishTest) {
  std::string channel_name = "drake_lcm_test_publisher_channel_name";

  MessageSubscriber subscriber(channel_name);

  // Instantes a test LCM message and serializes it into an array of bytes.
  drake::lcmt_drake_signal message;
  message.dim = 2;
  message.val.push_back(0.3739558136);
  message.val.push_back(0.2801694990);
  message.coord.push_back("artin's constant");
  message.coord.push_back("bernstein's constant");
  message.timestamp = 142857;

  uint8_t buffer[message.getEncodedSize()];
  EXPECT_EQ(message.encode(buffer, 0, message.getEncodedSize()),
            message.getEncodedSize());

  // Instantiates the Device Under Test (DUT).
  DrakeLcm dut;

  // Start the LCM recieve thread after all objects it can potentially use
  // are instantiated. Since objects are destructed in the reverse order of
  // construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the Lcm receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  LcmReceiveThread lcm_receive_thread(dut.get_lcm_instance());

  // Records whether the receiver received an LCM message published by the DUT.
  bool done = false;

  // Prevents this unit test from running indefinitely when the receiver fails
  // to receive the LCM message published by the DUT.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  // We must periodically call dut->Publish(...) since we do not know when
  // the receiver will actually receive the message.
  while (!done && count++ < kMaxCount) {
    dut->Publish(channel_name, buffer, message.getEncodedSize());

    // Gets the received message.
    const drake::lcmt_drake_signal received_message =
        subscriber.GetReceivedMessage();

    // Verifies that the received message equals the transmitted message.
    if (received_message.dim == message.dim) {
      bool values_match = true;

      if (received_message.timestamp != message.dim)
        values_match = false;

      for (int i = 0; i < message.dim && values_match; ++i) {
        if (received_message.val[i] != message.value[i]) values_match = false;
        if (received_message.coord[i] != message.coord[i]) values_match = false;
      }

      // At this point, if values_match is true the received message equals the
      // transmitted message, which means the DUT successfully published the
      // message.
      if (values_match) done = true;
    }

    if (!done) sleep_for(milliseconds(kDelayMS));
  }

  EXPECT_TRUE(done);
}

}  // namespace
}  // namespace lcm
}  // namespace drake

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"


namespace drake {
namespace systems {
namespace lcm {
namespace {

const int kDim = 10;
const int kPortNumber = 0;

/**
 * Subscribes to LCM messages of type `drake::lcmt_drake_signal`. Provides an
 * accessor to the latest message received.
 */
class MessageSubscriber {
 public:
  MessageSubscriber(const std::string& channel_name, ::lcm::LCM* lcm) :
      channel_name_(channel_name) {
    // Sets up the LCM message subscriber.
    ::lcm::Subscription* sub = lcm->subscribe(
      channel_name, &MessageSubscriber::HandleMessage, this);
    sub->setQueueCapacity(1);

    // Initializes the fields of member variable received_message_ so that the
    // test logic below can determine whether the desired message was received.
    received_message_.dim = 0;
    received_message_.val.resize(received_message_.dim);
    received_message_.coord.resize(received_message_.dim);
    received_message_.timestamp = 0;
  }

  drake::lcmt_drake_signal GetReceivedMessage() {
    drake::lcmt_drake_signal message_copy;

    message_mutex_.lock();
    message_copy = received_message_;
    message_mutex_.unlock();

    return message_copy;
  }

 private:
  void HandleMessage(const ::lcm::ReceiveBuffer *rbuf,
                     const std::string &channel_name) {
    if (channel_name_ == channel_name) {
      message_mutex_.lock();
      received_message_.decode(rbuf->data, 0, rbuf->data_size);
      message_mutex_.unlock();
    }
  }

  const std::string channel_name_;

  std::mutex message_mutex_;

  drake::lcmt_drake_signal received_message_;
};

// Tests the functionality of LcmPublisherSystem.
GTEST_TEST(LcmPublisherSystemTest, ReceiveTest) {
  ::lcm::LCM lcm;
  LcmReceiveThread lcm_receive_thread(&lcm);
  std::string channel_name = "drake_system2_lcm_test_publisher_channel_name";
  TranslatorBetweenLcmtDrakeSignal translator(kDim);

  // Instantiates an LcmPublisherSystem that takes as input System 2.0 Vectors
  // of type drake::systems::BasicVector and publishes LCM messages of type
  // drake::lcmt_drake_signal.
  //
  // It then verifies that the LcmPublisherSystem was successfully stored
  // in a unique_ptr. The unique_ptr variable is called "dut" to indicate it is
  // the "device under test".
  std::unique_ptr<LcmPublisherSystem> dut(
      new LcmPublisherSystem(channel_name, translator, &lcm));

  EXPECT_EQ(dut->get_name(), "LcmPublisherSystem::" + channel_name);

  // Instantiates a receiver of lcmt_drake_signal messages.
  MessageSubscriber subscriber(channel_name, &lcm);

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // Verifies that the context has one input port.
  EXPECT_EQ(context->get_num_input_ports(), 1);

  // Instantiates a BasicVector with known state. This is the basic vector that
  // we want the LcmPublisherSystem to publish as a drake::lcmt_drake_signal
  // message.
  std::unique_ptr<BasicVector<double>> basic_vector(
      new BasicVector<double>(kDim));

  {
    VectorX<double> vector_value;
    vector_value.resize(kDim);
    for (int ii = 0; ii < kDim; ++ii) {
      vector_value[ii] = ii;
    }
    basic_vector->set_value(vector_value);
  }

  // Sets the value in the context's input port to be the above-defined
  // BasicVector.
  std::unique_ptr<InputPort<double>> input_port(
    new FreestandingInputPort<double>(std::move(basic_vector)));

  context->SetInputPort(kPortNumber, std::move(input_port));

  // Whether the receiver received an LCM message published by the
  // LcmPublisherSystem.
  bool done = false;

  // This is used to prevent this unit test from running indefinitely when
  // the receiver fails to receive the LCM message published by the
  // LcmPublisherSystem.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  // We must periodically call dut->EvalOutput(...) since we do not know when
  // the receiver will receive the message published by the LcmPublisherSystem.
  while (!done && count++ < kMaxCount) {
    dut->EvalOutput(*context.get(), output.get());

    // Gets the received message.
    const drake::lcmt_drake_signal received_message =
        subscriber.GetReceivedMessage();

    // Verifies that the size of the received LCM message is correct.
    if (received_message.dim == kDim) {
      bool values_match = true;

      for (int ii = 0; ii < kDim && values_match; ++ii) {
        if (received_message.val[ii] != ii)
          values_match = false;
      }

      // At this point, the received message contains the expected values, which
      // implies that LcmPublisherSystem successfully published the BasicVector
      // as a drake::lcmt_drake_signal message.
      //
      // We cannot check whether the following member variables of
      // drake::lcmt_drake_signal message was successfully transferred because
      // BasicVector does not save this information:
      //
      //   1. coord
      //   2. timestamp
      //
      // Thus, we must conclude that the experiment succeeded.
      if (values_match) done = true;
    }

    if (!done) std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMS));
  }

  EXPECT_TRUE(done);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

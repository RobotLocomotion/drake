#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

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
  MessageSubscriber(const std::string& channel_name, ::lcm::LCM* lcm)
      : channel_name_(channel_name) {
    // Sets up the LCM message subscriber.
    ::lcm::Subscription* sub =
        lcm->subscribe(channel_name, &MessageSubscriber::HandleMessage, this);
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

    std::lock_guard<std::mutex> lock(message_mutex_);
    message_copy = received_message_;

    return message_copy;
  }

 private:
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

void TestPublisher(::lcm::LCM* lcm, const std::string& channel_name,
                   LcmPublisherSystem* dut) {
  EXPECT_EQ(dut->get_name(), "LcmPublisherSystem(" + channel_name + ")");

  // Instantiates a receiver of lcmt_drake_signal messages.
  MessageSubscriber subscriber(channel_name, lcm);

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput(*context);

  // Verifies that the context has one input port.
  EXPECT_EQ(context->get_num_input_ports(), 1);

  // Instantiates a BasicVector with known state. This is the basic vector that
  // we want the LcmPublisherSystem to publish as a drake::lcmt_drake_signal
  // message.
  std::unique_ptr<BasicVector<double>> vec(
      new BasicVector<double>(kDim));

  {
    Eigen::VectorBlock<VectorX<double>> vector_value =
        vec->get_mutable_value();

    for (int i = 0; i < kDim; ++i) {
      vector_value[i] = i;
    }
  }

  // Sets the value in the context's input port to be the above-defined
  // VectorBase. Note that we need to overwrite the original input port
  // created by the LcmPublisherSystem since we do not have write access to its
  // input vector.
  std::unique_ptr<InputPort> input_port(
      new FreestandingInputPort(std::move(vec)));

  context->SetInputPort(kPortNumber, std::move(input_port));

  // Sets the timestamp within the context. This timestamp should be transmitted
  // by the LCM message.
  const double time = 1.41421356;
  context->set_time(time);

  // Start the LCM recieve thread after all objects it can potentially use
  // are instantiated. Since objects are destructed in the reverse order of
  // construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the Lcm receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  LcmReceiveThread lcm_receive_thread(lcm);

  // Whether the receiver received an LCM message published by the
  // LcmPublisherSystem.
  bool done = false;

  // This is used to prevent this unit test from running indefinitely when
  // the receiver fails to receive the LCM message published by the
  // LcmPublisherSystem.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  // We must periodically call dut->Publish(...) since we do not know when
  // the receiver will receive the message published by the LcmPublisherSystem.
  while (!done && count++ < kMaxCount) {
    dut->Publish(*context.get());

    // Gets the received message.
    const drake::lcmt_drake_signal received_message =
        subscriber.GetReceivedMessage();

    // Verifies that the size of the received LCM message is correct.
    if (received_message.dim == kDim) {
      bool values_match = true;

      if (received_message.timestamp != static_cast<int64_t>(time * 1000))
        values_match = false;

      for (int i = 0; i < kDim && values_match; ++i) {
        if (received_message.val[i] != i) values_match = false;
      }

      // At this point, if values_match is true, the received message contains
      // the expected timestamp and values, which implies that
      // LcmPublisherSystem successfully published the VectorBase as a
      // drake::lcmt_drake_signal message.
      //
      // We cannot check whether drake::lcmt_drake_signal::coord was
      // successfully transferred because drake::systems::BasicVector does not
      // hold this information.
      //
      // Thus, we must conclude that the experiment succeeded.
      if (values_match) done = true;
    }

    if (!done) std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMS));
  }

  EXPECT_TRUE(done);
}

// Tests LcmPublisherSystem using a translator.
GTEST_TEST(LcmPublisherSystemTest, PublishTest) {
  ::lcm::LCM lcm;
  std::string channel_name = "drake_system2_lcm_test_publisher_channel_name";
  LcmtDrakeSignalTranslator translator(kDim);

  // Instantiates an LcmPublisherSystem that takes as input System 2.0 Vectors
  // of type drake::systems::VectorBase and publishes LCM messages of type
  // drake::lcmt_drake_signal.
  //
  // The LcmPublisherSystem is called "dut" to indicate it is the
  // "device under test".
  LcmPublisherSystem dut(channel_name, translator, &lcm);

  TestPublisher(&lcm, channel_name, &dut);
}

// Tests LcmPublisherSystem using a dictionary that contains a translator.
GTEST_TEST(LcmPublisherSystemTest, PublishTestUsingDictionary) {
  ::lcm::LCM lcm;
  std::string channel_name = "drake_system2_lcm_test_publisher_channel_name";

  // Creates a dictionary with one translator.
  LcmTranslatorDictionary dictionary;
  dictionary.AddEntry(
      channel_name,
      std::make_unique<const LcmtDrakeSignalTranslator>(kDim));

  EXPECT_TRUE(dictionary.HasTranslator(channel_name));

  // Instantiates an LcmPublisherSystem that takes as input System 2.0 Vectors
  // of type drake::systems::VectorBase and publishes LCM messages of type
  // drake::lcmt_drake_signal.
  //
  // The LcmPublisherSystem is called "dut" to indicate it is the
  // "device under test".
  LcmPublisherSystem dut(channel_name, dictionary, &lcm);

  TestPublisher(&lcm, channel_name, &dut);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

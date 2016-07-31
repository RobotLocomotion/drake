#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace systems {
namespace lcm {
namespace {

const int kDim = 10;
const int64_t kTimestamp = 123456;

/**
 * Periodically publishes an LCM message of type `drake::lcmt_drake_signal`.
 */
class MessagePublisher {
 public:
  MessagePublisher(const std::string& channel_name, ::lcm::LCM* lcm)
      : channel_name_(channel_name), lcm_(lcm) {
    message_.dim = kDim;
    message_.val.resize(kDim);
    message_.coord.resize(kDim);
    for (int ii = 0; ii < kDim; ++ii) {
      message_.val[ii] = ii;
      message_.coord[ii] = "coord_" + std::to_string(ii);
    }
    message_.timestamp = kTimestamp;
  }

  void Start() {
    thread_.reset(new std::thread(&MessagePublisher::DoPublish, this));
  }

  void Stop() {
    stop_ = true;
    thread_->join();
  }

 private:
  void DoPublish() {
    while (!stop_) {
      lcm_->publish(channel_name_, &message_);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  const std::string channel_name_;

  ::lcm::LCM* lcm_;

  drake::lcmt_drake_signal message_;

  std::atomic<bool> stop_{false};

  std::unique_ptr<std::thread> thread_;
};

void TestSubscriber(::lcm::LCM* lcm, const std::string& channel_name,
   LcmSubscriberSystem* dut) {
  EXPECT_EQ(dut->get_name(), "LcmSubscriberSystem::" + channel_name);

  // Instantiates a publisher of lcmt_drake_signal messages on the LCM network.
  // network.
  MessagePublisher publisher(channel_name, lcm);
  publisher.Start();

  std::unique_ptr<ContextBase<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput(*context);

  // Start the LCM recieve thread after all objects it can potentially use
  // are instantiated. Since objects are destructed in the reverse order of
  // construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the Lcm receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  LcmReceiveThread lcm_receive_thread(lcm);

  // Whether the LcmSubscriberSystem successfully received an LCM message and
  // outputted it as a BasicVector.
  bool done = false;

  // This is used to prevent this unit test from running indefinitely when
  // the LcmSubscriberSystem fails to output a BasicVector.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  // We must periodically call dut->EvalOutput(...) since we do not know when
  // the LcmSubscriberSystem will receive the LCM message and thus return a
  // valid output.
  while (!done && count++ < kMaxCount) {
    dut->EvalOutput(*context.get(), output.get());

    // Gets the output of the LcmSubscriberSystem.
    const drake::systems::VectorInterface<double>* vector =
        output->get_port(0).get_vector_data();

    // Downcasts the output vector to be a pointer to a BasicVector.
    const BasicVector<double>& basic_vector =
        dynamic_cast<const BasicVector<double>&>(*vector);

    // Verifies that the size of the basic vector is correct.
    if (basic_vector.size() == kDim) {
      // Verifies that the values in the basic vector are correct.
      Eigen::VectorBlock<const VectorX<double>> value =
          basic_vector.get_value();

      bool values_match = true;

      for (int ii = 0; ii < kDim && values_match; ++ii) {
        if (value[ii] != ii) values_match = false;
      }

      // At this point, the basic vector contains the expected values, which
      // must have been delivered by the receipt of an drake::lcmt_drake_signal
      // message.
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

  publisher.Stop();
}

// Tests the functionality of LcmSubscriberSystem.
GTEST_TEST(LcmSubscriberSystemTest, ReceiveTest) {
  // Instantiates LCM.
  ::lcm::LCM lcm;

  // Defines a channel name.
  const std::string channel_name =
      "drake_system2_lcm_test_subscriber_channel_name";

  // Instantiates a LCM-VectorInterface translator.
  const TranslatorBetweenLcmtDrakeSignal translator(kDim);

  // Instantiates an LcmSubscriberSystem that receives LCM messages of type
  // drake::lcmt_drake_signal and outputs System 2.0 Vectors of type
  // drake::systems::BasicVector.
  //
  // The LcmSubscriberSystem is called "dut" to indicate it is the
  // "device under test".
  LcmSubscriberSystem dut(channel_name, translator, &lcm);

  TestSubscriber(&lcm, channel_name, &dut);
}

// Tests the functionality of LcmSubscriberSystem.
GTEST_TEST(LcmSubscriberSystemTest, ReceiveTestUsingDictionary) {
  // Instantiates LCM.
  ::lcm::LCM lcm;

  // Defines a channel name.
  const std::string channel_name =
      "drake_system2_lcm_test_subscriber_channel_name";

  // Creates a dictionary with one translator.
  LcmTranslatorDictionary dictionary;
  dictionary.AddEntry(channel_name,
    std::make_unique<const TranslatorBetweenLcmtDrakeSignal>(kDim));

  EXPECT_TRUE(dictionary.HasTranslator(channel_name));

  // Instantiates an LcmSubscriberSystem that receives LCM messages of type
  // drake::lcmt_drake_signal and outputs System 2.0 Vectors of type
  // drake::systems::BasicVector.
  //
  // The LcmSubscriberSystem is called "dut" to indicate it is the
  // "device under test".
  LcmSubscriberSystem dut(channel_name, dictionary, &lcm);

  TestSubscriber(&lcm, channel_name, &dut);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

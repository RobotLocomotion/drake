#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <array>
#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
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

  ~MessagePublisher() {
    EXPECT_TRUE(stop_);
    // Test cases are required to call Stop() before completing, but sometimes
    // fail to do so (e.g., if the test case raised an unexpected exception).
    // If that happens, we need to join the thread here, or else its destructor
    // will fail and confuse the gtest reporting of the earlier failures.
    if (!stop_) {
      Stop();
    }
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
  EXPECT_EQ(dut->get_name(), "LcmSubscriberSystem(" + channel_name + ")");

  // Instantiates a publisher of lcmt_drake_signal messages on the LCM network.
  // network.
  MessagePublisher publisher(channel_name, lcm);
  publisher.Start();

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
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
    const BasicVector<double>& basic_vector = *output->get_vector_data(0);

    // Verifies that the size of the basic vector is correct.
    if (basic_vector.size() == kDim) {
      // Verifies that the values in the basic vector are correct.
      Eigen::VectorBlock<const VectorX<double>> value =
          basic_vector.get_value();

      bool values_match = true;

      for (int i = 0; i < kDim && values_match; ++i) {
        if (value[i] != i) values_match = false;
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

  // Instantiates a LCM-VectorBase translator.
  const LcmtDrakeSignalTranslator translator(kDim);

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
  dictionary.AddEntry(
      channel_name,
      std::make_unique<const LcmtDrakeSignalTranslator>(kDim));

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

// A lcmt_drake_signal translator that preserves coordinate names.
class CustomDrakeSignalTranslator : public LcmAndVectorBaseTranslator {
 public:
  // An output vector type that tests that subscribers permit non-default
  // types.
  class CustomVector : public BasicVector<double> {
   public:
    CustomVector() : BasicVector<double>(kDim) {}
    void SetName(int index, std::string name) { names_.at(index) = name; }
    std::string GetName(int index) const { return names_.at(index); }
   private:
    std::array<std::string, kDim> names_;
  };

  CustomDrakeSignalTranslator() : LcmAndVectorBaseTranslator(kDim) {}

  std::unique_ptr<BasicVector<double>> AllocateOutputVector() const override {
    return std::make_unique<CustomVector>();
  }

  void Deserialize(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const override {
    CustomVector* const custom_vector =
        dynamic_cast<CustomVector*>(vector_base);
    ASSERT_NE(nullptr, custom_vector);

    // Decode the LCM message.
    drake::lcmt_drake_signal message;
    int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
    ASSERT_GE(status, 0);
    ASSERT_EQ(kDim, message.dim);

    // Copy message into our custom_vector.
    for (int i = 0; i < kDim; ++i) {
      custom_vector->SetAtIndex(i, message.val[i]);
      custom_vector->SetName(i, message.coord[i]);
    }
  }

  void Serialize(double time,
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override {
    const CustomVector* const custom_vector =
        dynamic_cast<const CustomVector*>(&vector_base);
    ASSERT_NE(nullptr, custom_vector);
    ASSERT_NE(nullptr, lcm_message_bytes);

    // Copy custom_vector into the message.
    drake::lcmt_drake_signal message;
    message.dim = kDim;
    message.val.resize(kDim);
    message.coord.resize(kDim);
    message.timestamp = static_cast<int64_t>(time * 1000);

    for (int i = 0; i < kDim; ++i) {
      message.val.at(i) = custom_vector->GetAtIndex(i);
      message.coord.at(i) = custom_vector->GetName(i);
    }

    // Encode the LCM message.
    const int lcm_message_length = message.getEncodedSize();
    lcm_message_bytes->resize(lcm_message_length);
    message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
  }
};

// Subscribe and output a custom VectorBase type.
GTEST_TEST(LcmSubscriberSystemTest, CustomVectorBaseTest) {
  // The "device under test" and its prerequisites.
  CustomDrakeSignalTranslator translator;
  ::lcm::LCM lcm;
  LcmSubscriberSystem dut("dummy", translator, &lcm);

  // Create a data-filled vector.
  typedef CustomDrakeSignalTranslator::CustomVector CustomVector;
  CustomVector sample_vector;
  for (int i = 0; i < kDim; ++i) {
    sample_vector.SetAtIndex(i, i);
    sample_vector.SetName(i, std::to_string(i) + "_name");
  }

  // Set message into the dut.  It is encoded into bytes internally, which lets
  // us confirm that the full round-trip encode / decode cycle is correct.
  const double time = 0;
  dut.SetMessage(time, sample_vector);

  // Read back the vector via EvalOutput.
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);
  dut.EvalOutput(*context, output.get());
  const VectorBase<double>* const output_vector = output->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);

  // Check that the sample values match.
  const CustomVector* const custom_output =
      dynamic_cast<const CustomVector*>(output_vector);
  ASSERT_NE(nullptr, custom_output);
  for (int i = 0; i < kDim; ++i) {
    EXPECT_EQ(sample_vector.GetAtIndex(i), custom_output->GetAtIndex(i));
    EXPECT_EQ(sample_vector.GetName(i), custom_output->GetName(i));
  }
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

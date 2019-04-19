#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <array>
#include <future>

#include <gtest/gtest.h>

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/lcm/test/custom_drake_signal_translator.h"

using drake::lcm::CompareLcmtDrakeSignalMessages;

namespace drake {
namespace systems {
namespace lcm {
namespace {

constexpr int kDim = 10;
constexpr int64_t kTimestamp = 123456;

void EvalOutputHelper(const LcmSubscriberSystem& sub, Context<double>* context,
                      SystemOutput<double>* output) {
  auto event_info = sub.AllocateCompositeEventCollection();
  sub.CalcNextUpdateTime(*context, event_info.get());

  if (event_info->HasEvents()) {
    std::unique_ptr<State<double>> tmp_state = context->CloneState();
    if (event_info->HasDiscreteUpdateEvents()) {
      sub.CalcDiscreteVariableUpdates(
          *context, event_info->get_discrete_update_events(),
          &tmp_state->get_mutable_discrete_state());
    } else if (event_info->HasUnrestrictedUpdateEvents()) {
      sub.CalcUnrestrictedUpdate(*context,
          event_info->get_unrestricted_update_events(), tmp_state.get());
    } else {
      DRAKE_DEMAND(false);
    }
    context->get_mutable_state().SetFrom(*tmp_state);
  }
  sub.CalcOutput(*context, output);
}

void TestSubscriber(drake::lcm::DrakeMockLcm* lcm,
                    const std::string& channel_name, LcmSubscriberSystem* dut) {
  EXPECT_EQ(dut->get_name(), "LcmSubscriberSystem(" + channel_name + ")");

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  drake::lcmt_drake_signal message;
  message.dim = kDim;
  message.val.resize(kDim);
  message.coord.resize(kDim);
  for (int i = 0; i < kDim; ++i) {
    message.val[i] = i;
    message.coord[i] = "coord_" + std::to_string(i);
  }
  message.timestamp = kTimestamp;

  Publish(lcm, dut->get_channel_name(), message);
  lcm->HandleSubscriptions(0);

  EvalOutputHelper(*dut, context.get(), output.get());

  const BasicVector<double>& basic_vector = *output->get_vector_data(0);
  EXPECT_EQ(basic_vector.size(), kDim);
  Eigen::VectorBlock<const VectorX<double>> value = basic_vector.get_value();

  for (int i = 0; i < kDim; ++i) {
    EXPECT_EQ(value[i], i);
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Confirm that the unit test sugar used by pydrake is another equally-valid
  // way to read messages.
  auto new_context = dut->CreateDefaultContext();
  dut->CopyLatestMessageInto(&new_context->get_mutable_state());
  const auto& new_y = dut->get_output_port().Eval(*new_context);
  for (int i = 0; i < kDim; ++i) {
    EXPECT_EQ(new_y[i], i);
  }
#pragma GCC diagnostic pop
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Tests the functionality of LcmSubscriberSystem.
GTEST_TEST(LcmSubscriberSystemTest, ReceiveTest) {
  // Instantiates LCM.
  drake::lcm::DrakeMockLcm lcm;

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
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Tests the functionality of LcmSubscriberSystem.
GTEST_TEST(LcmSubscriberSystemTest, ReceiveTestUsingDictionary) {
  // Instantiates LCM.
  drake::lcm::DrakeMockLcm lcm;

  // Defines a channel name.
  const std::string channel_name =
      "drake_system2_lcm_test_subscriber_channel_name";

  // Creates a dictionary with one translator.
  LcmTranslatorDictionary dictionary;
  dictionary.AddEntry(channel_name,
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
#pragma GCC diagnostic pop

struct SampleData {
  lcmt_drake_signal value{2, {1.0, 2.0}, {"x", "y"}, 12345};

  void MockPublish(
      drake::lcm::DrakeMockLcm* lcm, const std::string& channel_name) const {
    Publish(lcm, channel_name, value);
    lcm->HandleSubscriptions(0);
  }
};

// Tests LcmSubscriberSystem using a Serializer.
GTEST_TEST(LcmSubscriberSystemTest, SerializerTest) {
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";

  // The "device under test".
  auto dut = LcmSubscriberSystem::Make<lcmt_drake_signal>(channel_name, &lcm);

  // Establishes the context and output for the dut.
  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // MockLcm produces a sample message.
  SampleData sample_data;
  sample_data.MockPublish(&lcm, channel_name);

  // Verifies that the dut produces the output message.
  EvalOutputHelper(*dut, context.get(), output.get());

  const AbstractValue* abstract_value = output->get_data(0);
  ASSERT_NE(abstract_value, nullptr);
  auto value = abstract_value->get_value<lcmt_drake_signal>();
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(value, sample_data.value));
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Tests LcmSubscriberSystem using a fixed-size Serializer.
GTEST_TEST(LcmSubscriberSystemTest, FixedSizeSerializerTest) {
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";
  const SampleData sample_data;

  // The "device under test".
  auto dut = LcmSubscriberSystem::MakeFixedSize(
      sample_data.value, channel_name, &lcm);

  // Establishes the context and output for the dut.
  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // MockLcm produces a sample message.
  sample_data.MockPublish(&lcm, channel_name);

  // Verifies that the dut produces the output message.
  EvalOutputHelper(*dut, context.get(), output.get());

  const AbstractValue* abstract_value = output->get_data(0);
  ASSERT_NE(abstract_value, nullptr);
  auto value = abstract_value->get_value<lcmt_drake_signal>();
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(value, sample_data.value));

  // Smaller messages should also work.
  SampleData smaller_data;
  smaller_data.value = lcmt_drake_signal{1, {1.0}, {"x"}, 12345};
  smaller_data.MockPublish(&lcm, channel_name);
  EvalOutputHelper(*dut, context.get(), output.get());
  const AbstractValue* small_abstract_value = output->get_data(0);
  ASSERT_NE(small_abstract_value, nullptr);
  auto small_value = small_abstract_value->get_value<lcmt_drake_signal>();
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(small_value, smaller_data.value));
}
#pragma GCC diagnostic pop

GTEST_TEST(LcmSubscriberSystemTest, WaitTest) {
  // Ensure that `WaitForMessage` works as expected.
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";

  // Start device under test, with background LCM thread running.
  auto dut = LcmSubscriberSystem::Make<lcmt_drake_signal>(channel_name, &lcm);

  SampleData sample_data;

  // Use simple atomic rather than condition variables, as it simplifies this
  // implementation.
  std::atomic<bool> started{};
  auto wait = [&]() {
    while (!started.load()) {}
  };

  // Test explicit value.
  started = false;
  auto future_count = std::async(std::launch::async, [&]() {
    EXPECT_EQ(dut->GetInternalMessageCount(), 0);
    started = true;
    return dut->WaitForMessage(0);
  });
  wait();
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_GE(future_count.get(), 1);

  // Test implicit value, retrieving message as well.
  started = false;
  auto future_message = std::async(std::launch::async, [&]() {
    int old_count = dut->GetInternalMessageCount();
    started = true;
    Value<lcmt_drake_signal> message;
    dut->WaitForMessage(old_count, &message);
    return message.get_value();
  });
  wait();
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(
      future_message.get(), sample_data.value));

  // Test WaitForMessageTimeout, when no message is sent
  int old_count = dut->GetInternalMessageCount();
  started = false;
  auto timeout_count = std::async(std::launch::async, [&]() {
    started = true;
    return dut->WaitForMessage(old_count, nullptr, 0.01 /** 10 ms */);
  });
  wait();
  // Expect a timeout, since no message has been sent
  EXPECT_EQ(timeout_count.get(), old_count);

  // Reset atomic and test WaitForMessageTimeout, with a message
  // Note: this generates a race condition between the timeout and the receive
  // thread. Success relies on the probability of failure being extremely small,
  // but it is theoretically possible for WaitForMessageTimeout to timeout
  // before the message is received, leading to test failure.
  started = false;
  auto second_timeout_count = std::async(std::launch::async, [&]() {
    EXPECT_EQ(dut->GetInternalMessageCount(), old_count);
    started = true;
    return dut->WaitForMessage(old_count, nullptr, 0.02 /** 20 ms */);
  });
  wait();
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_GE(second_timeout_count.get(), old_count + 1);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Subscribe and output a custom VectorBase type.
GTEST_TEST(LcmSubscriberSystemTest, CustomVectorBaseTest) {
  const std::string kChannelName = "dummy";

  // The "device under test" and its prerequisites.
  test::CustomDrakeSignalTranslator translator;
  drake::lcm::DrakeMockLcm lcm;
  LcmSubscriberSystem dut(kChannelName, translator, &lcm);

  // Create a data-filled vector.
  using CustomVector = test::CustomDrakeSignalTranslator::CustomVector;
  CustomVector sample_vector;
  for (int i = 0; i < sample_vector.size(); ++i) {
    sample_vector.SetAtIndex(i, i);
  }

  // Induce a message transmission so we can evaluate whether the LCM
  // subscriber was able to successfully decode the message.
  std::vector<uint8_t> message_bytes;
  translator.Serialize(0.0 /* time */, sample_vector, &message_bytes);
  lcm.InduceSubscriberCallback(kChannelName, &message_bytes[0],
                               message_bytes.size());

  // Read back the vector via CalcOutput.
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput();
  EvalOutputHelper(dut, context.get(), output.get());
  const VectorBase<double>* const output_vector = output->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);

  // Check that the sample values match.
  const CustomVector* const custom_output =
      dynamic_cast<const CustomVector*>(output_vector);
  ASSERT_NE(nullptr, custom_output);
  for (int i = 0; i < sample_vector.size(); ++i) {
    EXPECT_EQ(sample_vector.GetAtIndex(i), custom_output->GetAtIndex(i));
  }
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

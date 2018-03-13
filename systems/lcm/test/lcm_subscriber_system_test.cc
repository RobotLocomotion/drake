#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <array>
#include <future>

#include <gtest/gtest.h>

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

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
    context->get_mutable_state().CopyFrom(*tmp_state);
  }
  sub.CalcOutput(*context, output);
}

void TestSubscriber(drake::lcm::DrakeMockLcm* lcm,
                    const std::string& channel_name, LcmSubscriberSystem* dut) {
  EXPECT_EQ(dut->get_name(), "LcmSubscriberSystem(" + channel_name + ")");

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput(*context);

  drake::lcmt_drake_signal message;
  message.dim = kDim;
  message.val.resize(kDim);
  message.coord.resize(kDim);
  for (int i = 0; i < kDim; ++i) {
    message.val[i] = i;
    message.coord[i] = "coord_" + std::to_string(i);
  }
  message.timestamp = kTimestamp;

  std::vector<uint8_t> buffer(message.getEncodedSize());
  EXPECT_EQ(message.encode(&buffer[0], 0, message.getEncodedSize()),
            message.getEncodedSize());

  lcm->InduceSubscriberCallback(dut->get_channel_name(), &buffer[0],
                                message.getEncodedSize());

  EvalOutputHelper(*dut, context.get(), output.get());

  const BasicVector<double>& basic_vector = *output->get_vector_data(0);
  EXPECT_EQ(basic_vector.size(), kDim);
  Eigen::VectorBlock<const VectorX<double>> value = basic_vector.get_value();

  for (int i = 0; i < kDim; ++i) {
    EXPECT_EQ(value[i], i);
  }
}

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

  lcm.StartReceiveThread();
  TestSubscriber(&lcm, channel_name, &dut);
}

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

  lcm.StartReceiveThread();
  TestSubscriber(&lcm, channel_name, &dut);
}

struct SampleData {
  const lcmt_drake_signal value{2, {1.0, 2.0}, {"x", "y"}, 12345};

  void MockPublish(
      drake::lcm::DrakeMockLcm* lcm, const std::string& channel_name) {
    const int num_bytes = value.getEncodedSize();
    std::vector<uint8_t> buffer(num_bytes);
    value.encode(buffer.data(), 0, num_bytes);
    lcm->InduceSubscriberCallback(channel_name, buffer.data(), num_bytes);
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
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput(*context);

  // MockLcm produces a sample message.
  lcm.StartReceiveThread();
  SampleData sample_data;
  sample_data.MockPublish(&lcm, channel_name);

  // Verifies that the dut produces the output message.
  EvalOutputHelper(*dut, context.get(), output.get());

  const AbstractValue* abstract_value = output->get_data(0);
  ASSERT_NE(abstract_value, nullptr);
  auto value = abstract_value->GetValueOrThrow<lcmt_drake_signal>();
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(value, sample_data.value));
}

GTEST_TEST(LcmSubscriberSystemTest, WaitTest) {
  // Ensure that `WaitForMessage` works as expected.
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";

  // Start device under test, with background LCM thread running.
  auto dut = LcmSubscriberSystem::Make<lcmt_drake_signal>(channel_name, &lcm);
  lcm.StartReceiveThread();

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
}

// A lcmt_drake_signal translator that preserves coordinate names.
class CustomDrakeSignalTranslator : public LcmAndVectorBaseTranslator {
 public:
  // An output vector type that tests that subscribers permit non-default
  // types.
  using CustomVector = MyVector<kDim, double>;

  CustomDrakeSignalTranslator() : LcmAndVectorBaseTranslator(kDim) {}

  std::unique_ptr<BasicVector<double>> AllocateOutputVector() const override {
    return std::make_unique<CustomVector>();
  }

  static std::string NameFor(int index) {
    return std::to_string(index) + "_name";
  }

  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
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
      EXPECT_EQ(message.coord[i], NameFor(i));
      custom_vector->SetAtIndex(i, message.val[i]);
    }
  }

  void Serialize(double time, const VectorBase<double>& vector_base,
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
      message.coord.at(i) = NameFor(i);
    }

    // Encode the LCM message.
    const int lcm_message_length = message.getEncodedSize();
    lcm_message_bytes->resize(lcm_message_length);
    message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
  }
};

// Subscribe and output a custom VectorBase type.
GTEST_TEST(LcmSubscriberSystemTest, CustomVectorBaseTest) {
  const std::string kChannelName = "dummy";

  // The "device under test" and its prerequisites.
  CustomDrakeSignalTranslator translator;
  drake::lcm::DrakeMockLcm lcm;
  LcmSubscriberSystem dut(kChannelName, translator, &lcm);
  lcm.StartReceiveThread();

  // Create a data-filled vector.
  typedef CustomDrakeSignalTranslator::CustomVector CustomVector;
  CustomVector sample_vector;
  for (int i = 0; i < kDim; ++i) {
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
  auto output = dut.AllocateOutput(*context);
  EvalOutputHelper(dut, context.get(), output.get());
  const VectorBase<double>* const output_vector = output->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);

  // Check that the sample values match.
  const CustomVector* const custom_output =
      dynamic_cast<const CustomVector*>(output_vector);
  ASSERT_NE(nullptr, custom_output);
  for (int i = 0; i < kDim; ++i) {
    EXPECT_EQ(sample_vector.GetAtIndex(i), custom_output->GetAtIndex(i));
  }
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace lcm {
namespace {

const int kDim = 10;
const int kPortNumber = 0;

using drake::lcm::CompareLcmtDrakeSignalMessages;
using drake::lcm::DrakeMockLcm;

void TestPublisher(const std::string& channel_name, lcm::DrakeMockLcm* lcm,
                   LcmPublisherSystem* dut) {
  EXPECT_EQ(dut->get_name(), "LcmPublisherSystem(" + channel_name + ")");

  unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut->AllocateOutput(*context);

  // Verifies that the context has one input port.
  EXPECT_EQ(context->get_num_input_ports(), 1);

  // Instantiates a BasicVector with known state. This is the basic vector that
  // we want the LcmPublisherSystem to publish as a drake::lcmt_drake_signal
  // message.
  auto vec = make_unique<BasicVector<double>>(kDim);
  Eigen::VectorBlock<VectorX<double>> vector_value =
      vec->get_mutable_value();

  for (int i = 0; i < kDim; ++i) {
    vector_value[i] = i;
  }

  context->FixInputPort(kPortNumber, std::move(vec));

  // Sets the timestamp within the context. This timestamp should be transmitted
  // by the LCM message.
  const double time = 1.41421356;
  context->set_time(time);

  dut->Publish(*context.get());

  // Verifies that the correct message was published.
  const std::vector<uint8_t>& published_message_bytes =
      lcm->get_last_published_message(dut->get_channel_name());

  drake::lcmt_drake_signal received_message;
  received_message.decode(&published_message_bytes[0], 0,
      published_message_bytes.size());

  drake::lcmt_drake_signal expected_message;
  expected_message.dim = kDim;
  expected_message.val.resize(kDim);
  expected_message.coord.resize(kDim);
  for (int i = 0; i < kDim; ++i) {
    expected_message.val[i] = i;
    expected_message.coord[i] = "";
  }
  expected_message.timestamp = static_cast<int64_t>(time * 1000);

  // TODO(liang.fok) Replace this with a Google Test matcher.
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received_message,
                                             expected_message));
}

// Tests LcmPublisherSystem using a translator.
GTEST_TEST(LcmPublisherSystemTest, PublishTest) {
  lcm::DrakeMockLcm lcm;
  const std::string channel_name =
      "drake_system2_lcm_test_publisher_channel_name";
  LcmtDrakeSignalTranslator translator(kDim);

  // Instantiates an LcmPublisherSystem that takes as input System 2.0 Vectors
  // of type drake::systems::VectorBase and publishes LCM messages of type
  // drake::lcmt_drake_signal.
  //
  // The LcmPublisherSystem is called "dut" to indicate it is the
  // "device under test".
  LcmPublisherSystem dut(channel_name, translator, &lcm);

  TestPublisher(channel_name, &lcm, &dut);
}

// Tests LcmPublisherSystem using a dictionary that contains a translator.
GTEST_TEST(LcmPublisherSystemTest, PublishTestUsingDictionary) {
  lcm::DrakeMockLcm lcm;
  const std::string channel_name =
      "drake_system2_lcm_test_publisher_channel_name";

  // Creates a dictionary with one translator.
  LcmTranslatorDictionary dictionary;
  dictionary.AddEntry(
      channel_name,
      make_unique<const LcmtDrakeSignalTranslator>(kDim));

  EXPECT_TRUE(dictionary.HasTranslator(channel_name));

  // Instantiates an LcmPublisherSystem that takes as input System 2.0 Vectors
  // of type drake::systems::VectorBase and publishes LCM messages of type
  // drake::lcmt_drake_signal.
  //
  // The LcmPublisherSystem is called "dut" to indicate it is the
  // "device under test".
  LcmPublisherSystem dut(channel_name, dictionary, &lcm);

  TestPublisher(channel_name, &lcm, &dut);
}

// Tests LcmPublisherSystem using a Serializer.
GTEST_TEST(LcmPublisherSystemTest, SerializerTest) {
  lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";

  // The "device under test".
  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(channel_name, &lcm);
  ASSERT_NE(dut.get(), nullptr);

  // Establishes the context, output, and input for the dut.
  unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut->AllocateOutput(*context);
  const lcmt_drake_signal sample_data{
    2, { 1.0, 2.0, }, { "x", "y", }, 12345,
  };
  context->SetInputPortValue(
      kPortNumber, make_unique<FreestandingInputPortValue>(
          make_unique<Value<lcmt_drake_signal>>(sample_data)));

  // Verifies that a correct message is published.
  dut->Publish(*context.get());
  const auto& bytes = lcm.get_last_published_message(channel_name);
  drake::lcmt_drake_signal received_message{};
  received_message.decode(bytes.data(), 0, bytes.size());
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received_message, sample_data));
}

// Tests that the published LCM message has the expected timestamps.
GTEST_TEST(LcmPublisherSystemTest, TestPublishPeriod) {
  const double kPublishPeriod = 1.5;  // Seconds between publications.

  lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";
  LcmtDrakeSignalTranslator translator(kDim);

  // Instantiates the "device under test".
  auto dut = make_unique<LcmPublisherSystem>(channel_name, translator, &lcm);
  dut->set_publish_period(kPublishPeriod);
  unique_ptr<Context<double>> context = dut->AllocateContext();

  context->FixInputPort(kPortNumber,
      make_unique<BasicVector<double>>(Eigen::VectorXd::Zero(kDim)));

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(*dut, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  for (double time = 0; time < 4; time += 0.01) {
    simulator.StepTo(time);
    EXPECT_NEAR(simulator.get_mutable_context()->get_time(), time, 1e-10);
    // Note that the expected time is in milliseconds.
    const double expected_time =
        std::floor(time / kPublishPeriod) * kPublishPeriod * 1000;
    EXPECT_EQ(lcm.DecodeLastPublishedMessageAs<lcmt_drake_signal>(
      channel_name).timestamp, expected_time);
  }
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

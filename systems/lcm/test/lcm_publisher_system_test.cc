#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/lcm/drake_lcm.h"
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
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeMockLcm;
using drake::lcm::DrakeLcm;

void TestPublisher(const std::string& channel_name, lcm::DrakeMockLcm* lcm,
                   LcmPublisherSystem* dut) {
  EXPECT_EQ(dut->get_name(), "LcmPublisherSystem(" + channel_name + ")");

  unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

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

  dut->PublishInputAsLcmMessage(*context.get());

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

  EXPECT_EQ(
      lcm->get_last_publication_time(dut->get_channel_name()).value_or(-1.0),
      time);
}

// Test that failure to specify an LCM interface results in an internal one
// of being allocated. Can't check for operation in this case
// since we won't have a mock LCM to look at.
GTEST_TEST(LcmPublisherSystemTest, DefaultLcmTest) {
  const std::string channel_name = "junk";

  // Use a translator just so we can invoke a constructor.
  LcmtDrakeSignalTranslator translator(kDim);

  // Provide an explicit LCM interface and check that it gets used.
  DrakeMockLcm mock_lcm;
  LcmPublisherSystem dut1(channel_name, translator, &mock_lcm);
  EXPECT_EQ(&dut1.lcm(), &mock_lcm);

  // Now leave out the LCM interface and check that a DrakeLcm gets allocated.
  LcmPublisherSystem dut2(channel_name, translator, nullptr);
  EXPECT_TRUE(is_dynamic_castable<DrakeLcm>(&dut2.lcm()));
}

// Test that an initialization publisher gets invoked properly by an
// initialization event, and that the initialization event doesn't cause
// publishing.
GTEST_TEST(LcmPublisherSystemTest, TestInitializationEvent) {
  const std::string channel_name = "junk";

  // Use a translator just so we can invoke a constructor.
  LcmtDrakeSignalTranslator translator(kDim);

  DrakeMockLcm mock_lcm;
  LcmPublisherSystem dut1(channel_name, translator, &mock_lcm);

  bool init_was_called{false};
  dut1.AddInitializationMessage([&dut1, &init_was_called](
      const Context<double>&, DrakeLcmInterface* lcm) {
    EXPECT_EQ(lcm, &dut1.lcm());
    init_was_called = true;
  });

  auto context = dut1.AllocateContext();

  // Put something on the input port so that an attempt to publish would
  // succeed if (erroneously) attempted after initialization.
  auto vec = make_unique<BasicVector<double>>(kDim);
  for (int i = 0; i < kDim; ++i) (*vec)[i] = i;
  context->FixInputPort(kPortNumber, std::move(vec));

  // Get the initialization event and publish it (this is mocking
  // Simulator::Initialize() behavior.
  auto init_events = dut1.AllocateCompositeEventCollection();
  dut1.GetInitializationEvents(*context, &*init_events);
  dut1.Publish(*context, init_events->get_publish_events());

  EXPECT_TRUE(init_was_called);

  // Nothing should have been published to this channel.
  EXPECT_FALSE(mock_lcm.get_last_publication_time(channel_name));
}

// Tests LcmPublisherSystem using a translator.
GTEST_TEST(LcmPublisherSystemTest, PublishTest) {
  lcm::DrakeMockLcm lcm;
  const std::string channel_name =
      "drake_system2_lcm_test_publisher_channel_name";
  LcmtDrakeSignalTranslator translator(kDim);

  // Instantiates an LcmPublisherSystem that takes as input of type
  // drake::systems::VectorBase and publishes LCM messages of type
  // drake::lcmt_drake_signal.
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

  // Instantiates an LcmPublisherSystem that takes as input of type
  // drake::systems::VectorBase and publishes LCM messages of type
  // drake::lcmt_drake_signal.
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
  unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();
  const lcmt_drake_signal sample_data{
    2, { 1.0, 2.0, }, { "x", "y", }, 12345,
  };
  context->FixInputPort(kPortNumber,
                        make_unique<Value<lcmt_drake_signal>>(sample_data));

  // Verifies that a correct message is published.
  dut->PublishInputAsLcmMessage(*context.get());
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
    EXPECT_NEAR(simulator.get_mutable_context().get_time(), time, 1e-10);
    // Note that the expected time is in milliseconds.
    const double expected_time =
        std::floor(time / kPublishPeriod) * kPublishPeriod * 1000;
    EXPECT_EQ(lcm.DecodeLastPublishedMessageAs<lcmt_drake_signal>(
      channel_name).timestamp, expected_time);
  }
}

GTEST_TEST(LcmPublisherSystemTest, OwnedTranslatorTest) {
  lcm::DrakeMockLcm lcm;
  const std::string channel_name = "my_channel";
  auto translator = std::make_unique<LcmtDrakeSignalTranslator>(kDim);

  LcmPublisherSystem dut(channel_name, std::move(translator), &lcm);

  TestPublisher(channel_name, &lcm, &dut);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

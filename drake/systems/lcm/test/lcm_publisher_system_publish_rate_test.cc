#include "drake/systems/lcm/lcm_publisher_system.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
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

using drake::lcm::DrakeMockLcm;

// Tests ability to set and get the desired publishing period.
GTEST_TEST(LcmPublisherSystemPublishRateTest, TestGetSetPeriod) {
  lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";

  // The "device under test".
  unique_ptr<LcmPublisherSystem> dut =
      LcmPublisherSystem::Make<lcmt_drake_signal>(channel_name, &lcm);
  ASSERT_NE(dut.get(), nullptr);

  unique_ptr<Context<double>> context = dut->AllocateContext();
  UpdateActions<double> update_actions;

  // Verifies that the publishing period is infinity at time 0.
  context->set_time(0);
  dut->CalcNextUpdateTime(*context, &update_actions);
  EXPECT_EQ(update_actions.time, std::numeric_limits<double>::infinity());

  // Verifies that the publishing period is infinity at time > 0.
  context->set_time(0.6875);
  dut->CalcNextUpdateTime(*context, &update_actions);
  EXPECT_EQ(update_actions.time, std::numeric_limits<double>::infinity());

  // Verifies that LcmPublisherSystem::CalcNextUpdateTime() sets the correct
  // next-publish-time at simulation time 0.
  const double kPeriod = 0.25;
  dut->set_publish_period(kPeriod);
  context->set_time(0);
  dut->CalcNextUpdateTime(*context, &update_actions);
  EXPECT_EQ(update_actions.time, kPeriod);

  // Verifies that LcmPublisherSystem::CalcNextUpdateTime() sets the correct
  // next-publish-time at simulation time > 0.
  const double kSimTime = 68.34;
  context->set_time(kSimTime);
  dut->CalcNextUpdateTime(*context, &update_actions);

  const int64_t num_periods = static_cast<int64_t>(ceil(kSimTime / kPeriod));
  double next_t = num_periods * kPeriod;
  if (next_t <= kSimTime)
    next_t = (num_periods + 1) * kPeriod;
  DRAKE_ASSERT(next_t > kSimTime);
  EXPECT_EQ(update_actions.time, next_t);
}

// Verifies that the last transmitted message's timestamp is equal to the
// provided timestamp.
void VerifyTimestamp(const std::vector<uint8_t>& transmitted_message_bytes,
                     double timestamp) {
  lcmt_drake_signal transmitted_message;
  // Decodes message and checks that the correct number of bytes was processed.
  EXPECT_EQ(transmitted_message.decode(transmitted_message_bytes.data(), 0,
                           transmitted_message_bytes.size()),
            transmitted_message_bytes.size());
  EXPECT_EQ(transmitted_message.timestamp, timestamp);
}

// Tests that the published LCM message has the expected timestamps.
GTEST_TEST(LcmPublisherSystemPublishRateTest, TestPublishPeriod) {
  const int kDim = 10;
  const int kPortNumber = 0;
  const double kPublishPeriod = 1.5;  // Seconds between publications.

  lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";
  LcmtDrakeSignalTranslator translator(kDim);

  // Instantiates the "device under test".
  auto dut = make_unique<LcmPublisherSystem>(channel_name, translator, &lcm);
  dut->set_publish_period(kPublishPeriod);
  std::unique_ptr<Context<double>> context = dut->AllocateContext();

  auto vec = make_unique<BasicVector<double>>(kDim);
  Eigen::VectorBlock<VectorX<double>> vector_value =
      vec->get_mutable_value();

  for (int i = 0; i < kDim; ++i) {
    vector_value[i] = i;
  }

  context->FixInputPort(kPortNumber, std::move(vec));

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(*dut, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(*dut,
                                              simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);
  simulator.get_mutable_integrator()->set_minimum_step_size(1e-3);
  simulator.get_mutable_integrator()->set_maximum_step_size(1e-3);
  simulator.Initialize();

  // Initially, the simulation time is 0 and the timstamp of the last
  // published LCM message was at time 0.
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 0);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 0);

  // At time 0.1, the last published LCM message remains at time 0.
  simulator.StepTo(0.1);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 0.1);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 0);

  // At time 1.0, the last published LCM message remains at time 0.
  simulator.StepTo(1.0);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 1.0);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 0);

  // At time 1.49, the last published LCM message remains at time 0.
  simulator.StepTo(1.49);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 1.49);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 0);

  // At time 1.5, the last published LCM message is at time 1.5.
  simulator.StepTo(1.5);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 1.5);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 1.5 * 1000);

  // At time 2.0, the last published LCM message is at time 1.5.
  simulator.StepTo(2.0);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 2.0);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 1.5 * 1000);

  // At time 2.99, the last published LCM message is at time 1.5.
  simulator.StepTo(2.99);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 2.99);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 1.5 * 1000);

  // At time 3.0, the last published LCM message is at time 3.0.
  simulator.StepTo(3.0);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), 3.0);
  VerifyTimestamp(lcm.get_last_published_message(channel_name), 3.0 * 1000);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

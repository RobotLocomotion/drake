#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <array>
#include <future>

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

using drake::lcm::CompareLcmtDrakeSignalMessages;

namespace drake {
namespace systems {
namespace lcm {
namespace {

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

struct SampleData {
  lcmt_drake_signal value{2, {1.0, 2.0}, {"x", "y"}, 12345};

  void PublishAndHandle(
      drake::lcm::DrakeLcmInterface* lcm,
      const std::string& channel_name) const {
    Publish(lcm, channel_name, value);
    lcm->HandleSubscriptions(0);
  }
};

// Tests the forced update handler of LcmSubscriberSystem.
GTEST_TEST(LcmSubscriberSystemTest, ForcedEventTest) {
  drake::lcm::DrakeLcm lcm;
  const std::string channel_name = "channel_name";

  // The "device under test".
  auto dut = LcmSubscriberSystem::Make<lcmt_drake_signal>(channel_name, &lcm);

  // Establish the context and output for the dut.
  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // Produce a sample message.
  SampleData sample_data;
  sample_data.PublishAndHandle(&lcm, channel_name);

  // Call the forced update handler to update the abstract states.
  std::unique_ptr<State<double>> tmp_state = context->CloneState();
  dut->CalcUnrestrictedUpdate(*context, tmp_state.get());
  context->get_mutable_state().SetFrom(*tmp_state);
  dut->CalcOutput(*context, output.get());

  const AbstractValue* abstract_value = output->get_data(0);
  ASSERT_NE(abstract_value, nullptr);
  auto value = abstract_value->get_value<lcmt_drake_signal>();
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(value, sample_data.value));
}

// Tests LcmSubscriberSystem using a Serializer.
GTEST_TEST(LcmSubscriberSystemTest, ReceiveTest) {
  drake::lcm::DrakeLcm lcm;
  const std::string channel_name = "channel_name";

  // The "device under test".
  auto dut = LcmSubscriberSystem::Make<lcmt_drake_signal>(channel_name, &lcm);

  // Establish the context and output for the dut.
  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // Produce a sample message.
  SampleData sample_data;
  sample_data.PublishAndHandle(&lcm, channel_name);

  // Verify that the dut produces the output message.
  EvalOutputHelper(*dut, context.get(), output.get());

  const AbstractValue* abstract_value = output->get_data(0);
  ASSERT_NE(abstract_value, nullptr);
  auto value = abstract_value->get_value<lcmt_drake_signal>();
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(value, sample_data.value));
}

GTEST_TEST(LcmSubscriberSystemTest, WaitTest) {
  // Ensure that `WaitForMessage` works as expected.
  drake::lcm::DrakeLcm lcm;
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
  sample_data.PublishAndHandle(&lcm, channel_name);
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
  sample_data.PublishAndHandle(&lcm, channel_name);
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
  sample_data.PublishAndHandle(&lcm, channel_name);
  EXPECT_GE(second_timeout_count.get(), old_count + 1);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace lcm {
namespace {

using drake::lcm::CompareLcmtDrakeSignalMessages;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcm;

using Subscriber = drake::lcm::Subscriber<lcmt_drake_signal>;

// Test that failure to specify an LCM interface results in an internal one
// being allocated.
GTEST_TEST(LcmPublisherSystemTest, DefaultLcmTest) {
  const std::string channel_name = "junk";

  // Provide an explicit LCM interface and check that it gets used.
  DrakeLcm interface;
  auto dut1 = LcmPublisherSystem::Make<lcmt_drake_signal>(
      channel_name, &interface);
  EXPECT_EQ(&(dut1->lcm()), &interface);

  // Now leave out the LCM interface and check that a DrakeLcm gets allocated.
  auto dut2 = LcmPublisherSystem::Make<lcmt_drake_signal>(
      channel_name, nullptr);
  EXPECT_TRUE(is_dynamic_castable<DrakeLcm>(&(dut2->lcm())));
}

// Test that an initialization publisher gets invoked properly by an
// initialization event, and that the initialization event doesn't cause
// publishing.
GTEST_TEST(LcmPublisherSystemTest, TestInitializationEvent) {
  const std::string channel_name = "junk";

  DrakeLcm interface;
  auto dut1 = LcmPublisherSystem::Make<lcmt_drake_signal>(
      channel_name, &interface);
  Subscriber sub(&interface, channel_name);

  bool init_was_called{false};
  dut1->AddInitializationMessage([&interface, &init_was_called](
      const Context<double>&, DrakeLcmInterface* lcm) {
    EXPECT_EQ(lcm, &interface);
    init_was_called = true;
  });

  auto context = dut1->AllocateContext();

  // Put something on the input port so that an attempt to publish would
  // succeed if (erroneously) attempted after initialization.
  dut1->get_input_port().FixValue(context.get(), lcmt_drake_signal{});

  // Get the initialization event and publish it (this is mocking
  // Simulator::Initialize() behavior.
  auto init_events = dut1->AllocateCompositeEventCollection();
  dut1->GetInitializationEvents(*context, &*init_events);
  dut1->Publish(*context, init_events->get_publish_events());

  EXPECT_TRUE(init_was_called);

  // Nothing should have been published to this channel.
  interface.HandleSubscriptions(0);
  EXPECT_EQ(sub.count(), 0);
}

// Tests LcmPublisherSystem using a Serializer.
GTEST_TEST(LcmPublisherSystemTest, SerializerTest) {
  lcm::DrakeLcm interface;
  const std::string channel_name = "channel_name";

  // The "device under test".
  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(
      channel_name, &interface);
  ASSERT_NE(dut.get(), nullptr);

  // Establishes the context, output, and input for the dut.
  unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();
  const lcmt_drake_signal sample_data{
    2, { 1.0, 2.0, }, { "x", "y", }, 12345,
  };
  dut->get_input_port().FixValue(context.get(), sample_data);

  // Verifies that a correct message is published.
  Subscriber sub(&interface, channel_name);
  dut->Publish(*context.get());
  interface.HandleSubscriptions(0);
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(sub.message(), sample_data));
}

// Tests that per-step publish generates the expected number of publishes.
GTEST_TEST(LcmPublisherSystemTest, TestPerStepPublish) {
  lcm::DrakeLcm interface;
  const std::string channel_name = "channel_name";
  Subscriber sub(&interface, channel_name);

  // Instantiate the "device under test" in per-step publishing mode.
  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(
      channel_name, &interface);
  unique_ptr<Context<double>> context = dut->AllocateContext();
  dut->get_input_port().FixValue(context.get(), lcmt_drake_signal{});

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(*dut, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.Initialize();

  // Check that a message was transmitted during initialization.
  interface.HandleSubscriptions(0);
  EXPECT_EQ(sub.count(), 1);

  // Ensure that the integrator takes at least a few steps.
  for (double time = 0; time < 1; time += 0.25) {
    simulator.AdvanceTo(time);
    interface.HandleSubscriptions(0);
  }

  // Check that we get exactly the number of publishes desired: one (at
  // initialization) plus another for each step.
  EXPECT_EQ(sub.count(), 1 + simulator.get_num_steps_taken());
}

// When constructed via a publish_triggers set, tests that per-step publish
// generates the expected number of publishes.
GTEST_TEST(LcmPublisherSystemTest, TestPerStepPublishTrigger) {
  lcm::DrakeLcm interface;
  const std::string channel_name = "channel_name";
  Subscriber sub(&interface, channel_name);

  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(channel_name,
      &interface, {TriggerType::kPerStep});

  unique_ptr<Context<double>> context = dut->AllocateContext();
  dut->get_input_port().FixValue(context.get(), lcmt_drake_signal{});

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(*dut, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.Initialize();

  // Check that a message was transmitted during initialization.
  interface.HandleSubscriptions(0);
  EXPECT_EQ(sub.count(), 1);

  // Ensure that the integrator takes at least a few steps.
  // Since there is no internal continuous state for the system, the integrator
  // will not subdivide its steps.
  for (double time = 0.0; time < 1; time += 0.25) {
    simulator.AdvanceTo(time);
    interface.HandleSubscriptions(0);
  }

  // Check that we get exactly the number of publishes desired: one (at
  // initialization) plus another for each step.
  EXPECT_EQ(sub.count(), 1 + simulator.get_num_steps_taken());
}

// When constructed via a publish_triggers set, tests that forced publish
// generates the expected number of publishes.
GTEST_TEST(LcmPublisherSystemTest, TestForcedPublishTrigger) {
  lcm::DrakeLcm interface;
  const std::string channel_name = "channel_name";
  int force_publish_count = 3;
  Subscriber sub(&interface, channel_name);

  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(channel_name,
      &interface, {TriggerType::kForced});

  unique_ptr<Context<double>> context = dut->AllocateContext();
  dut->get_input_port().FixValue(context.get(), lcmt_drake_signal{});

  for (int i = 0; i < force_publish_count; i++) {
    dut->Publish(*context);
    interface.HandleSubscriptions(0);
  }

  // Check that we get exactly the number of publishes desired.
  EXPECT_EQ(sub.count(), force_publish_count);
}

class TimeMessageSystem final : public LeafSystem<double> {
 public:
  TimeMessageSystem() {
    this->DeclareAbstractOutputPort("output", &TimeMessageSystem::CalcOutput);
  }

 private:
  void CalcOutput(const Context<double>& context,
                  lcmt_drake_signal* output) const {
    *output = {};
    output->timestamp = static_cast<int64_t>(context.get_time() * 1000);
  }
};

// Tests that the published LCM message has the expected timestamps.
GTEST_TEST(LcmPublisherSystemTest, TestPublishPeriod) {
  const double kPublishPeriod = 1.5;  // Seconds between publications.

  lcm::DrakeLcm interface;
  const std::string channel_name = "channel_name";
  Subscriber sub(&interface, channel_name);

  // Cascade a time source with the device under test.
  DiagramBuilder<double> builder;
  auto source = builder.AddSystem<TimeMessageSystem>();
  auto dut = builder.AddSystem(LcmPublisherSystem::Make<lcmt_drake_signal>(
      channel_name, &interface, kPublishPeriod));
  builder.Connect(*source, *dut);
  auto diagram = builder.Build();

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.Initialize();

  // Check that a message was transmitted during initialization.
  interface.HandleSubscriptions(0);
  EXPECT_EQ(sub.count(), 1);

  for (double time = 0; time < 4; time += 0.01) {
    simulator.AdvanceTo(time);
    interface.HandleSubscriptions(0);
    EXPECT_NEAR(simulator.get_mutable_context().get_time(), time, 1e-10);
    // Note that the expected time is in milliseconds.
    const double expected_time =
        std::floor(time / kPublishPeriod) * kPublishPeriod * 1000;
    EXPECT_EQ(sub.message().timestamp, expected_time);
  }

  // Check that we get the expected number of messages: one at initialization
  // plus two from periodic publishing.
  EXPECT_EQ(sub.count(), 3);
}

// When constructed via a publish_triggers set, tests that periodic publishing
// generates the expected number of publishes.
GTEST_TEST(LcmPublisherSystemTest, TestPublishPeriodTrigger) {
  const double kPublishPeriod = 1.5;  // Seconds between publications.

  lcm::DrakeLcm interface;
  const std::string channel_name = "channel_name";
  Subscriber sub(&interface, channel_name);

  // Instantiates the "device under test".
  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(channel_name,
      &interface, {TriggerType::kPeriodic},
      kPublishPeriod);
  unique_ptr<Context<double>> context = dut->AllocateContext();
  dut->get_input_port().FixValue(context.get(), lcmt_drake_signal{});

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(*dut, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.Initialize();

  // Check that a message was transmitted during initialization.
  interface.HandleSubscriptions(0);
  EXPECT_EQ(sub.count(), 1);

  for (double time = 0.0; time < 4; time += 0.01) {
    simulator.AdvanceTo(time);
    interface.HandleSubscriptions(0);
  }

  // Check that we get the expected number of messages: one at initialization
  // plus two from periodic publishing.
  EXPECT_EQ(sub.count(), 3);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

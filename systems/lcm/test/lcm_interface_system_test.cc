#include "drake/systems/lcm/lcm_interface_system.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using systems::DiagramBuilder;

class LcmInterfaceSystemTest : public ::testing::TestWithParam<int> {};

// In a sender thread, transmit a message every 10ms with some timestamp.  In
// the simulation, check that the timestamps evolve monotonically.
TEST_P(LcmInterfaceSystemTest, AcceptanceTest) {
  DiagramBuilder<double> builder;
  std::unique_ptr<DrakeLcm> drake_lcm;

  // The device under test is the LcmInterfaceSystem.  We test all three
  // constructor varieties.
  LcmInterfaceSystem* dut{};
  const int param = GetParam();
  DRAKE_DEMAND((param >= 0) && (param <= 2));
  switch (param) {
    case 0: {
      dut = builder.AddSystem<LcmInterfaceSystem>();
      break;
    }
    case 1: {
      dut = builder.AddSystem<LcmInterfaceSystem>("memq://");
      break;
    }
    case 2: {
      drake_lcm = std::make_unique<DrakeLcm>();
      dut = builder.AddSystem<LcmInterfaceSystem>(drake_lcm.get());
      break;
    }
  }
  DRAKE_DEMAND(dut != nullptr);

  // Publish using our external LCM if it exists, otherwise use the DUT.
  DrakeLcmInterface* publisher_lcm = drake_lcm.get();
  if (!publisher_lcm) {
    publisher_lcm = dut;
  }

  // Smoke test the HandleSubscriptions forwarding.
  EXPECT_EQ(dut->HandleSubscriptions(0), 0);

  // Add a subscriber so that we can see messages being pumped.
  const std::string channel("test_channel");
  auto subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>(channel, dut));
  builder.ExportOutput(subscriber->get_output_port());
  auto diagram = builder.Build();
  const auto& message_port = diagram->get_output_port(0);

  // Transmit a timestamp, in a background thread.
  std::atomic<int64_t> next_tx_timestamp{0};
  auto thread = std::make_unique<std::thread>(
      [&next_tx_timestamp, &publisher_lcm, channel](){
        while (next_tx_timestamp >= 0) {
          lcmt_drake_signal message{};
          message.timestamp = next_tx_timestamp;
          Publish(publisher_lcm, channel, message);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });

  // Simulate and watch the timestamp go up.
  auto simulator = std::make_unique<Simulator<double>>(std::move(diagram));
  simulator->set_target_realtime_rate(1.0);
  int64_t last_rx_timestamp{0};
  while (last_rx_timestamp < 100) {
    const auto& context = simulator->get_context();
    // The specific dt here doesn't matter; we just want to sleep a little
    // bit sometimes to avoid a busy-loop.
    const double dt = 0.0025;
    simulator->AdvanceTo(context.get_time() + dt);
    const auto& message = message_port.Eval<lcmt_drake_signal>(context);
    EXPECT_GE(message.timestamp, last_rx_timestamp);
    last_rx_timestamp = message.timestamp;
    next_tx_timestamp = last_rx_timestamp + 1;
  }

  if (drake_lcm) {
    // If we made our own DrakeLcm instead of letting LcmSubscriberSystem make
    // one, then we should be able to delete the simulator and its systems and
    // still publish and service subscriptions.  The LcmSubscriberSystem's
    // subscriptions should NOT be called anymore.
    simulator.reset();
    // Empty the queue of anything that was published subsequent to the final
    // simulator.AdvanceTo call, but prior to the simulator.reset call.
    drake_lcm->HandleSubscriptions(0);
    // Even though the transmit thread is still publishing, nothing else should
    // appear at the handlers because there are no more active subscriptions.
    int total = 0;
    for (int i = 0; i < 100; ++i) {
      total += drake_lcm->HandleSubscriptions(1);
    }
    // Nothing was subscribed anymore, so no messages should have been handled.
    EXPECT_EQ(total, 0);
  }

  next_tx_timestamp = -1;
  thread->join();
  thread.reset();
}

// Test that multiple LcmInterfaceSystems on the same URL do not cause a name
// collision.
TEST_F(LcmInterfaceSystemTest, NameCollisionTest) {
  DiagramBuilder<double> builder;
  builder.AddSystem<LcmInterfaceSystem>();
  builder.AddSystem<LcmInterfaceSystem>();
  builder.Build();
}

INSTANTIATE_TEST_SUITE_P(test, LcmInterfaceSystemTest,
                        ::testing::Values(0, 1, 2));

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

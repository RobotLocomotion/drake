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
using systems::DiagramBuilder;

// In a sender thread, transmit a message every 10ms with some timestamp.  In
// the simulation, check that the timestamps evolve monotonically.
GTEST_TEST(LcmInterfaceSystemTest, AcceptanceTest) {
  // All LCM traffic will go through here.
  DrakeLcm lcm_real;

  // The device under test is the LcmInterfaceSystem.
  DiagramBuilder<double> builder;
  auto dut = builder.AddSystem<LcmInterfaceSystem>(&lcm_real);

  // Add a subscriber so that we can see messages being pumped.
  const std::string channel("test_channel");
  auto subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>(channel, dut));
  builder.ExportOutput(subscriber->get_output_port());
  auto diagram = builder.Build();
  const auto& message_port = diagram->get_output_port(0);

  // Transmit a timestamp, in a background thread.
  std::atomic<int64_t> next_tx_timestamp{0};
  auto thread = std::make_unique<std::thread>([&](){
    while (next_tx_timestamp >= 0) {
      lcmt_drake_signal message{};
      message.timestamp = next_tx_timestamp;
      Publish(&lcm_real, channel, message);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  // Simulate and watch the timestamp go up.
  Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  int64_t last_rx_timestamp{0};
  while (last_rx_timestamp < 100) {
    simulator.StepTo(simulator.get_context().get_time() + 0.001);
    const auto& context = simulator.get_context();
    const auto& message = message_port.Eval<lcmt_drake_signal>(context);
    EXPECT_GE(message.timestamp, last_rx_timestamp);
    last_rx_timestamp = message.timestamp;
    next_tx_timestamp = last_rx_timestamp + 1;
  }

  next_tx_timestamp = -1;
  thread->join();
  thread.reset();
}

// TODO(jwnimmer-tri) Value-param test with a non-memq URL.

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

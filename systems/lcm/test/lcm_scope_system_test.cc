#include "drake/systems/lcm/lcm_scope_system.h"

#include <gtest/gtest.h>

#include "drake/lcmt_scope.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

// Scope a Sine and check the output.
GTEST_TEST(ScopeTest, Acceptance) {
  // Set up the diagram.
  const std::string channel = "CHANNEL";
  const double publish_period = 0.25;
  DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<LcmInterfaceSystem>();
  auto source = builder.AddSystem<Sine>(
      Eigen::Vector2d(1.0,        2.0),          // amplitudes
      Eigen::Vector2d(2.0 * M_PI, 2 * M_PI),     // frequencies = 1 rev / sec
      Eigen::Vector2d(0.0,        0.5 * M_PI));  // phases
  auto [scope, publisher] = LcmScopeSystem::AddToBuilder(
      &builder, lcm, source->get_output_port(0), channel, publish_period);
  ASSERT_NE(scope, nullptr);
  ASSERT_NE(publisher, nullptr);

  // Simulate and listen for received messages.
  drake::lcm::Subscriber<lcmt_scope> subscriber(lcm, channel);
  Simulator<double> simulator(builder.Build());

  // When time == 0.0.
  simulator.AdvanceTo(0.0);
  lcm->HandleSubscriptions(0);
  ASSERT_EQ(subscriber.count(), 1);
  ASSERT_EQ(subscriber.message().size, 2);
  EXPECT_NEAR(subscriber.message().value[0], 0.0, 1e-10);
  EXPECT_NEAR(subscriber.message().value[1], 2.0, 1e-10);
  subscriber.clear();

  // When we approach time == publish_period (but don't cross it yet).
  simulator.AdvanceTo(0.99 * publish_period);
  lcm->HandleSubscriptions(0);
  ASSERT_EQ(subscriber.count(), 0);

  // When we reach time == publish_period exactly.
  simulator.AdvanceTo(publish_period);
  lcm->HandleSubscriptions(0);
  ASSERT_EQ(subscriber.count(), 1);
  ASSERT_EQ(subscriber.message().size, 2);
  EXPECT_NEAR(subscriber.message().value[0], 1.0, 1e-10);
  EXPECT_NEAR(subscriber.message().value[1], 0.0, 1e-10);
  subscriber.clear();
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

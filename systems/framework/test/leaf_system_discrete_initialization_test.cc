#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

using Eigen::VectorXd;
const double kEps = std::numeric_limits<double>::epsilon();
class Dut : public LeafSystem<double> {
 public:
  Dut() {
    initialized_index_ = DeclareDiscreteState(VectorXd::Zero(1));
    DeclareInitializationDiscreteUpdateEvent(&Dut::Initialize);
    DeclareVectorOutputPort(
        "initialized",
        BasicVector<double>(1),
        [this](const Context<double>& context, BasicVector<double>* output) {
          (*output)[0] = IsInitialized(context);
        });
  }

  bool IsInitialized(const Context<double>& context) const {
    const double initialized =
        context.get_discrete_state(initialized_index_)[0];
    return initialized != 0.0;
  }

 private:
  EventStatus Initialize(
      const Context<double>& context,
      DiscreteValues<double>* state) const {
    state->get_mutable_value(initialized_index_)[0] = 1.0;
    return EventStatus::Succeeded();
  }

  DiscreteStateIndex initialized_index_;
};

GTEST_TEST(LeafSystemDiscreteInitializationTest, Behavior) {
  DiagramBuilder<double> builder;

  const double zoh_period_sec = 1.0;
  // Add Dut.
  auto* dut = builder.AddSystem<Dut>();
  // Add cascade of ZOH.
  // N.B. Initializing has no effect on this sequence of events.
  const bool initialize = true;
  auto* zoh_1 =
      builder.AddSystem<ZeroOrderHold>(zoh_period_sec, 1, initialize);
  auto* zoh_2 =
      builder.AddSystem<ZeroOrderHold>(zoh_period_sec, 1, initialize);
  auto* zoh_3 =
      builder.AddSystem<ZeroOrderHold>(zoh_period_sec, 1, initialize);
  // Connect 3 ZOH in series.
  const int zoh_cascade_order = 3;
  builder.Connect(dut->get_output_port(), zoh_1->get_input_port());
  builder.Connect(zoh_1->get_output_port(), zoh_2->get_input_port());
  builder.Connect(zoh_2->get_output_port(), zoh_3->get_input_port());
  // Discrete cascade order is one more due to our dut's initialization event.
  const int discrete_initialization_cascade_order = zoh_cascade_order + 1;

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  auto& diagram_context = simulator.get_mutable_context();
  const auto diagram_context_init = diagram_context.Clone();
  const auto& dut_context = dut->GetMyContextFromRoot(diagram_context);

  auto zoh_shows_initialized = [&](const System<double>* zoh) {
    const auto& zoh_context = zoh->GetMyContextFromRoot(diagram_context);
    return zoh_context.get_discrete_state(0)[0] == 1.0;
  };

  // Default context is as expected.
  EXPECT_FALSE(dut->IsInitialized(dut_context));
  EXPECT_FALSE(zoh_shows_initialized(zoh_1));
  EXPECT_FALSE(zoh_shows_initialized(zoh_2));
  EXPECT_FALSE(zoh_shows_initialized(zoh_3));

  // Initialize.
  simulator.Initialize();
  // Our Dut initalization was processed. Note that the ZOH discrete update at
  // t=0 is *not* an initialization event, thus was not processed.
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  EXPECT_FALSE(zoh_shows_initialized(zoh_1));
  EXPECT_FALSE(zoh_shows_initialized(zoh_2));
  EXPECT_FALSE(zoh_shows_initialized(zoh_3));

  // Advance.
  simulator.AdvanceTo(0.0);
  // As before, we have processed the initialization event.
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  // The ZOH updated, and the dut's y⁻(0) was initialized, thus
  // indicates initialization.
  EXPECT_TRUE(zoh_shows_initialized(zoh_1));
  // Subsequent ZOH values are not yet updated.
  EXPECT_FALSE(zoh_shows_initialized(zoh_2));
  EXPECT_FALSE(zoh_shows_initialized(zoh_3));

  // Advance to exactly the boundary. Process all events with
  // AdvancePendingEvents(). No change should occur.
  simulator.AdvanceTo(zoh_period_sec);
  simulator.AdvancePendingEvents();
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  EXPECT_TRUE(zoh_shows_initialized(zoh_1));
  EXPECT_TRUE(zoh_shows_initialized(zoh_2));
  EXPECT_FALSE(zoh_shows_initialized(zoh_3));

  // One more time, so that all are updated.
  simulator.AdvanceTo(zoh_period_sec * 2);
  simulator.AdvancePendingEvents();
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  EXPECT_TRUE(zoh_shows_initialized(zoh_1));
  EXPECT_TRUE(zoh_shows_initialized(zoh_2));
  EXPECT_TRUE(zoh_shows_initialized(zoh_3));

  // Reset and show we're back to initial state.
  diagram_context.SetTimeStateAndParametersFrom(*diagram_context_init);
  EXPECT_FALSE(dut->IsInitialized(dut_context));
  EXPECT_FALSE(zoh_shows_initialized(zoh_1));
  EXPECT_FALSE(zoh_shows_initialized(zoh_2));
  EXPECT_FALSE(zoh_shows_initialized(zoh_3));

  // Show that simultaneous discrete updates can be handled by precisely
  // calling `simulator.Initialize)` by a value related to degree of cascade by
  // showing ultimate consistency.
  for (int i = 0; i < discrete_initialization_cascade_order; ++i) {
    simulator.Initialize();
  }
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  EXPECT_TRUE(zoh_shows_initialized(zoh_1));
  EXPECT_TRUE(zoh_shows_initialized(zoh_2));
  EXPECT_TRUE(zoh_shows_initialized(zoh_3));
}

}  // namespace
}  // namespace systems
}  // namespace drake

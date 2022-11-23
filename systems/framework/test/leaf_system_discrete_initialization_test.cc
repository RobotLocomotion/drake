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
  void DoCalcNextUpdateTime(
      const Context<double>& context,
      CompositeEventCollection<double>* events,
      double* time) const override {
    LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
    DRAKE_DEMAND(!events->HasEvents());
    DRAKE_DEMAND(std::isinf(*time));
    if (IsInitialized(context)) {
      return;
    }
    // Schedule for now.
    *time = context.get_time();
    auto initialize = [this](
        const Context<double>& event_context,
        const DiscreteUpdateEvent<double>&,
        DiscreteValues<double>* result) {
      result->get_mutable_value(initialized_index_)[0] = 1.0;
    };
    events->get_mutable_discrete_update_events().AddEvent(
      DiscreteUpdateEvent<double>(initialize));
  }

  DiscreteStateIndex initialized_index_;
};

GTEST_TEST(LeafSystemDiscreteInitializationTest, Behavior) {
  DiagramBuilder<double> builder;

  auto* dut = builder.AddSystem<Dut>();
  const double zoh_period_sec = 1.0;
  auto* zoh = builder.AddSystem<ZeroOrderHold>(zoh_period_sec, 1);
  builder.Connect(dut->get_output_port(), zoh->get_input_port());

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  const auto& diagram_context = simulator.get_context();
  const auto& dut_context = dut->GetMyContextFromRoot(diagram_context);
  const auto& zoh_context = zoh->GetMyContextFromRoot(diagram_context);

  auto zoh_shows_initialized = [&]() {
    return zoh_context.get_discrete_state(0)[0] != 0.0;
  };

  // Default context is as expected.
  EXPECT_FALSE(dut->IsInitialized(dut_context));
  EXPECT_FALSE(zoh_shows_initialized());

  // Initialize.
  simulator.Initialize();
  // Note: Still false!
  EXPECT_FALSE(dut->IsInitialized(dut_context));
  EXPECT_FALSE(zoh_shows_initialized());

  // Advance.
  simulator.AdvanceTo(0.0);
  // Now it changes!
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  // ... but effectively dependent system has no state change upon
  // *initialization* event?
  EXPECT_FALSE(zoh_shows_initialized());

  // Advance to exactly the boundary.
  simulator.AdvanceTo(zoh_period_sec);
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  // Er...?
  EXPECT_FALSE(zoh_shows_initialized());

  // Advance again.
  simulator.AdvanceTo(zoh_period_sec + kEps);
  // Everything as expected.
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  EXPECT_TRUE(zoh_shows_initialized());
}

}  // namespace
}  // namespace systems
}  // namespace drake

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
    // TODO(eric.cousineau): Should this instead just be an initialization
    // event?
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
  // Note: Initialization() is a *specific* kind of event. Our manually
  // scheduled pseudo-initialization event is not detectable beyond nominal
  // event queuing.
  EXPECT_FALSE(dut->IsInitialized(dut_context));
  EXPECT_FALSE(zoh_shows_initialized());

  // Advance.
  simulator.AdvanceTo(0.0);
  // Now initialized as we have processed the t=0 pseudo-initialization event.
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  // The ZOH updated, but was operating on dut's y⁻(0) (not initialized), thus
  // indicates no initialization.
  EXPECT_FALSE(zoh_shows_initialized());

  // Advance to exactly the boundary. Process all events with
  // AdvancePendingEvents().
  simulator.AdvanceTo(zoh_period_sec);
  simulator.AdvancePendingEvents();
  EXPECT_TRUE(dut->IsInitialized(dut_context));
  EXPECT_TRUE(zoh_shows_initialized());
}

}  // namespace
}  // namespace systems
}  // namespace drake

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

using Eigen::VectorXd;

class Dut : public LeafSystem<double> {
 public:
  explicit Dut(double period_sec_) {
    // Initialization.
    initialized_index_ = DeclareDiscreteState(VectorXd::Zero(1));
    initialize_counter_index_ = DeclareDiscreteState(VectorXd::Zero(1));
    DeclareInitializationDiscreteUpdateEvent(&Dut::Initialize);
    DeclareStateOutputPort("initialized", initialized_index_);
    // Discrete counter.
    periodic_counter_index_ = DeclareDiscreteState(VectorXd::Zero(1));
    DeclarePeriodicDiscreteUpdateEvent(
        period_sec_, 0.0, &Dut::UpdatePeriodicCounter);
  }

  bool IsInitialized(const Context<double>& context) const {
    const double initialized =
        context.get_discrete_state(initialized_index_)[0];
    return initialized == 1.0;
  }

  int GetInitializeCounter(const Context<double>& context) const {
    const double counter =
        context.get_discrete_state(initialize_counter_index_)[0];
    return static_cast<int>(counter);
  }

  int GetPeriodicCounter(const Context<double>& context) const {
    const double counter =
        context.get_discrete_state(periodic_counter_index_)[0];
    return static_cast<int>(counter);
  }

 private:
  EventStatus Initialize(
      const Context<double>& context,
      DiscreteValues<double>* state) const {
    state->get_mutable_value(initialized_index_)[0] = 1.0;
    state->get_mutable_value(initialize_counter_index_)[0] += 1.0;
    return EventStatus::Succeeded();
  }

  void UpdatePeriodicCounter(
      const Context<double>& context,
      DiscreteValues<double>* state) const {
    state->get_mutable_value(periodic_counter_index_)[0] += 1.0;
  }

  DiscreteStateIndex initialized_index_;
  DiscreteStateIndex initialize_counter_index_;
  DiscreteStateIndex periodic_counter_index_;
};

class LeafSystemDiscreteInitializationTest : public ::testing::Test {
 public:
  LeafSystemDiscreteInitializationTest() { }

 protected:
  void BuildDiagram(bool zoh_initialize) {
    DiagramBuilder<double> builder;

    // Add Dut.
    dut_ = builder.AddSystem<Dut>(period_sec_);
    // Add cascade of ZOH.
    zoh_1_ = builder.AddSystem<ZeroOrderHold>(period_sec_, 1, zoh_initialize);
    zoh_2_ = builder.AddSystem<ZeroOrderHold>(period_sec_, 1, zoh_initialize);
    zoh_3_ = builder.AddSystem<ZeroOrderHold>(period_sec_, 1, zoh_initialize);
    // Connect 3 ZOH in series.
    builder.Connect(dut_->get_output_port(), zoh_1_->get_input_port());
    builder.Connect(zoh_1_->get_output_port(), zoh_2_->get_input_port());
    builder.Connect(zoh_2_->get_output_port(), zoh_3_->get_input_port());

    diagram_ = builder.Build();
    simulator_ = std::make_unique<Simulator<double>>(*diagram_);
    diagram_context_ = &simulator_->get_mutable_context();
    dut_context_ = &dut_->GetMyContextFromRoot(*diagram_context_);
  }

  void CheckDefaultContext() const {
    // Default context is as expected.
    EXPECT_FALSE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(dut_->GetInitializeCounter(*dut_context_), 0);
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 0);
    EXPECT_FALSE(IsZohInitialized(zoh_1_));
    EXPECT_FALSE(IsZohInitialized(zoh_2_));
    EXPECT_FALSE(IsZohInitialized(zoh_3_));
  }

  bool IsZohInitialized(const System<double>* zoh) const {
    const auto& zoh_context = zoh->GetMyContextFromRoot(*diagram_context_);
    return zoh_context.get_discrete_state(0)[0] == 1.0;
  }

  const double period_sec_ = 1.0;

  const Dut* dut_{};
  const ZeroOrderHold<double>* zoh_1_{};
  const ZeroOrderHold<double>* zoh_2_{};
  const ZeroOrderHold<double>* zoh_3_{};
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
  const Context<double>* diagram_context_{};
  const Context<double>* dut_context_{};
};

TEST_F(LeafSystemDiscreteInitializationTest, Nominal) {
  for (bool zoh_initialize : {false, true}) {
    SCOPED_TRACE(fmt::format("zoh_initialize = {}", zoh_initialize));
    BuildDiagram(zoh_initialize);
    CheckDefaultContext();

    // Initialize.
    simulator_->Initialize();
    // Our Dut initalization was processed. Note that the ZOH discrete update at
    // t=0 is *not* an initialization event, thus was not processed.
    EXPECT_TRUE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(dut_->GetInitializeCounter(*dut_context_), 1);
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 0);
    EXPECT_FALSE(IsZohInitialized(zoh_1_));
    EXPECT_FALSE(IsZohInitialized(zoh_2_));
    EXPECT_FALSE(IsZohInitialized(zoh_3_));

    // Advance.
    simulator_->AdvanceTo(0.0);
    // As before, we have processed the initialization event.
    EXPECT_TRUE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(dut_->GetInitializeCounter(*dut_context_), 1);
    // We have also processed one discrete update.
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 1);
    // The ZOH updated, and the dut_'s y⁻(0) was initialized, thus
    // indicates initialization.
    EXPECT_TRUE(IsZohInitialized(zoh_1_));
    // Subsequent ZOH values are not yet updated.
    EXPECT_FALSE(IsZohInitialized(zoh_2_));
    EXPECT_FALSE(IsZohInitialized(zoh_3_));

    // Advance to exactly the boundary. Process all events with
    // AdvancePendingEvents(). No change should occur.
    simulator_->AdvanceTo(period_sec_);
    simulator_->AdvancePendingEvents();
    EXPECT_TRUE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(dut_->GetInitializeCounter(*dut_context_), 1);
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 2);
    EXPECT_TRUE(IsZohInitialized(zoh_1_));
    EXPECT_TRUE(IsZohInitialized(zoh_2_));
    EXPECT_FALSE(IsZohInitialized(zoh_3_));

    // One more time, so that all are updated.
    simulator_->AdvanceTo(period_sec_ * 2);
    simulator_->AdvancePendingEvents();
    EXPECT_TRUE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(dut_->GetInitializeCounter(*dut_context_), 1);
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 3);
    EXPECT_TRUE(IsZohInitialized(zoh_1_));
    EXPECT_TRUE(IsZohInitialized(zoh_2_));
    EXPECT_TRUE(IsZohInitialized(zoh_3_));
  }
}

TEST_F(LeafSystemDiscreteInitializationTest, RepeatedInitialize) {
  // Show that simultaneous discrete updates can be handled by precisely
  // calling `simulator_->Initialize()` by order of cascade through showing
  // converged behavior for discrete initialization events.

  // Note: Given the use of the initialization counter, we don't have a fixed
  // point the strictest of senses. However, this is only meant for testing /
  // inspection.
  // Users wanting to use this trick should ideally:
  // - Not encode "non-fixed-point beahvior" for discrete intialialization
  //   events
  // - Avoid any external effects in those events. Instead, defer them to
  //   (periodic) publish events.

  // Discrete cascade order is one more due to our dut_'s initialization event.
  const int zoh_cascade_order = 3;
  const int discrete_initialization_cascade_order = zoh_cascade_order + 1;

  // Show that this does *not* work for zoh_initialize = false.
  {
    SCOPED_TRACE("zoh_initialize = false");
    const bool zoh_initialize = false;
    BuildDiagram(zoh_initialize);
    CheckDefaultContext();

    for (int i = 0; i < discrete_initialization_cascade_order; ++i) {
      simulator_->Initialize();
    }

    EXPECT_TRUE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 0);
    EXPECT_EQ(
        dut_->GetInitializeCounter(*dut_context_),
        discrete_initialization_cascade_order);
    EXPECT_FALSE(IsZohInitialized(zoh_1_));
    EXPECT_FALSE(IsZohInitialized(zoh_2_));
    EXPECT_FALSE(IsZohInitialized(zoh_3_));
  }

  // Show that it does work for zoh_initialize = true.
  {
    SCOPED_TRACE("zoh_initialize = true");
    const bool zoh_initialize = true;
    BuildDiagram(zoh_initialize);
    CheckDefaultContext();

    for (int i = 0; i < discrete_initialization_cascade_order; ++i) {
      simulator_->Initialize();
    }

    EXPECT_TRUE(dut_->IsInitialized(*dut_context_));
    EXPECT_EQ(
        dut_->GetInitializeCounter(*dut_context_),
        discrete_initialization_cascade_order);
    EXPECT_EQ(dut_->GetPeriodicCounter(*dut_context_), 0);
    EXPECT_TRUE(IsZohInitialized(zoh_1_));
    EXPECT_TRUE(IsZohInitialized(zoh_2_));
    EXPECT_TRUE(IsZohInitialized(zoh_3_));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

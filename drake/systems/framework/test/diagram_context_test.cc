#include "drake/systems/framework/diagram_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {
namespace {

constexpr double kTime = 12.0;

class DiagramContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder0_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));
    adder0_->set_name("adder0");
    adder1_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));
    adder1_->set_name("adder1");
    adder2_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));
    adder2_->set_name("adder2");

    integrator0_.reset(new Integrator<double>(1 /* length */));
    integrator1_.reset(new Integrator<double>(1 /* length */));

    context_.reset(new DiagramContext<double>());
    context_->set_time(kTime);

    AddSystem(*adder0_, 0);
    AddSystem(*adder1_, 1);
    AddSystem(*integrator0_, 2);
    AddSystem(*integrator1_, 3);

    context_->ExportInput({0 /* adder0_ */, 1 /* port 1 */});
    context_->ExportInput({1 /* adder1_ */, 0 /* port 0 */});

    context_->MakeState();
    ContinuousState<double>* xc = context_->get_state().continuous_state.get();
    xc->get_mutable_state()->SetAtIndex(0, 42.0);
    xc->get_mutable_state()->SetAtIndex(1, 43.0);
  }

  void AddSystem(const System<double>& sys, int index) {
    auto subcontext = sys.CreateDefaultContext();
    auto suboutput = sys.AllocateOutput(*subcontext);
    context_->AddSystem(index, std::move(subcontext), std::move(suboutput));
  }

  void AttachInputPorts() {
    auto vec0 = std::make_unique<BasicVector<double>>(std::vector<double>{128});
    auto vec1 = std::make_unique<BasicVector<double>>(std::vector<double>{256});

    auto port0 =
        std::make_unique<FreestandingInputPort<double>>(std::move(vec0));
    auto port1 =
        std::make_unique<FreestandingInputPort<double>>(std::move(vec1));
    inputs_.push_back(port0.get());
    inputs_.push_back(port1.get());
    context_->SetInputPort(0, std::move(port0));
    context_->SetInputPort(1, std::move(port1));
  }

  std::unique_ptr<AbstractValue> PackValue(int value) {
    return std::unique_ptr<AbstractValue>(new Value<int>(value));
  }

  int UnpackValue(const AbstractValue* value) {
    return dynamic_cast<const Value<int>*>(value)->get_value();
  }

  std::unique_ptr<DiagramContext<double>> context_;
  std::unique_ptr<Adder<double>> adder0_;
  std::unique_ptr<Adder<double>> adder1_;
  std::unique_ptr<Adder<double>> adder2_;
  std::unique_ptr<Integrator<double>> integrator0_;
  std::unique_ptr<Integrator<double>> integrator1_;

  std::vector<FreestandingInputPort<double>*> inputs_;
};

// Tests that systems do not have outputs or contexts in the DiagramContext
// until they are added as constituent systems.
TEST_F(DiagramContextTest, AddAndRetrieveConstituents) {
  EXPECT_EQ(nullptr, context_->GetSubsystemOutput(4));
  EXPECT_EQ(nullptr, context_->GetSubsystemContext(4));

  AddSystem(*adder2_, 4 /* index */);

  EXPECT_NE(nullptr, context_->GetSubsystemOutput(4));
  EXPECT_NE(nullptr, context_->GetSubsystemContext(4));
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(DiagramContextTest, State) {
  ContinuousState<double>* xc = context_->get_state().continuous_state.get();
  EXPECT_EQ(2, xc->get_state().size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());

  // Changes to the diagram state write through to constituent system states.
  ContinuousState<double>* integrator0_xc =
      context_->GetSubsystemContext(2)->get_state().continuous_state.get();
  ContinuousState<double>* integrator1_xc =
      context_->GetSubsystemContext(3)->get_state().continuous_state.get();
  EXPECT_EQ(42.0, integrator0_xc->get_state().GetAtIndex(0));
  EXPECT_EQ(43.0, integrator1_xc->get_state().GetAtIndex(0));

  // Changes to constituent system states appear in the diagram state.
  integrator1_xc->get_mutable_state()->SetAtIndex(0, 1000.0);
  EXPECT_EQ(1000.0, xc->get_state().GetAtIndex(1));
}

// Tests that no exception is thrown when connecting a valid source
// and destination port.
TEST_F(DiagramContextTest, ConnectValid) {
  EXPECT_NO_THROW(context_->Connect({0 /* adder0_ */, 0 /* port 0 */},
                                    {1 /* adder1_ */, 1 /* port 1 */}));
}

// Tests that an exception is thrown when connecting from a source port that
// does not exist.
TEST_F(DiagramContextTest, ConnectInvalidSrcPort) {
  EXPECT_THROW(context_->Connect({0 /* adder0_ */, 1 /* port 1 */},
                                 {1 /* adder1_ */, 1 /* port 1 */}),
               std::out_of_range);
}

// Tests that an exception is thrown when connecting to a destination port that
// does not exist.
TEST_F(DiagramContextTest, ConnectInvalidDestPort) {
  EXPECT_THROW(context_->Connect({0 /* adder0_ */, 0 /* port 0 */},
                                 {1 /* adder1_ */, 2 /* port 2 */}),
               std::out_of_range);
}

// Tests that input ports can be assigned to the DiagramContext and then
// retrieved.
TEST_F(DiagramContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(2, context_->get_num_input_ports());
  AttachInputPorts();
  EXPECT_EQ(128, context_->get_vector_input(0)->get_value()[0]);
  EXPECT_EQ(256, context_->get_vector_input(1)->get_value()[0]);
}

// Tests that an exception is thrown when setting or getting input ports that
// don't exist.
TEST_F(DiagramContextTest, InvalidInputPorts) {
  EXPECT_THROW(context_->SetInputPort(2, nullptr), std::out_of_range);
  EXPECT_THROW(context_->get_vector_input(2), std::out_of_range);
}

TEST_F(DiagramContextTest, Clone) {
  context_->Connect({0 /* adder0_ */, 0 /* port 0 */},
                    {1 /* adder1_ */, 1 /* port 1 */});
  AttachInputPorts();

  std::unique_ptr<DiagramContext<double>> clone(
      dynamic_cast<DiagramContext<double>*>(context_->Clone().release()));
  ASSERT_NE(nullptr, clone);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state was copied.
  ContinuousState<double>* xc = context_->get_state().continuous_state.get();

  EXPECT_EQ(42.0, xc->get_state().GetAtIndex(0));
  EXPECT_EQ(43.0, xc->get_state().GetAtIndex(1));

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(2, clone->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    EXPECT_NE(context_->get_vector_input(i), clone->get_vector_input(i));
    EXPECT_TRUE(CompareMatrices(context_->get_vector_input(i)->get_value(),
                                clone->get_vector_input(i)->get_value(), 1e-8,
                                MatrixCompareType::absolute));
  }

  // Verify that the graph structure was preserved: the VectorInterface in
  // sys0 output port 0 should be pointer-equal to the VectorInterface in
  // sys1 input port 1.
  EXPECT_EQ(clone->GetSubsystemContext(1)->get_vector_input(1),
            clone->GetSubsystemOutput(0)->get_port(0).get_vector_data());
}

class DiagramContextCacheInvalidationTest : public DiagramContextTest {
 protected:
  void SetUp() override {
    DiagramContextTest::SetUp();
    AttachInputPorts();
    adder0_context_ = dynamic_cast<Context<double>*>(
        const_cast<ContextBase<double>*>(context_->GetSubsystemContext(0)));
  }

  Context<double>* adder0_context_;
};

// Tests that cache invalidation on an input port is propagated down to the
// affected subsystems.
TEST_F(DiagramContextCacheInvalidationTest,
       InputPortCacheInvalidationPropagates) {
  ContextShape<bool> dependencies = adder0_context_->MakeContextShape();
  ASSERT_EQ(2u, dependencies.input_ports.size());
  // Input port 0 of the Diagram is input port 1 of adder0_.
  dependencies.input_ports[1] = true;
  CacheTicket ticket = adder0_context_->CreateCacheEntry(dependencies, {});

  adder0_context_->SetCachedItem(ticket, PackValue(42));
  EXPECT_EQ(42, UnpackValue(adder0_context_->GetCachedItem(ticket)));
  context_->InvalidateInputPort(0);
  EXPECT_EQ(nullptr, adder0_context_->GetCachedItem(ticket));
}

}  // namespace
}  // namespace systems
}  // namespace drake

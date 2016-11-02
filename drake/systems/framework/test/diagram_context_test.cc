#include "drake/systems/framework/diagram_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {
namespace {

constexpr int kNumSystems = 4;
constexpr double kTime = 12.0;

class DiagramContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder0_.reset(new Adder<double>(2 /* inputs */, 1 /* size */));
    adder0_->set_name("adder0");
    adder1_.reset(new Adder<double>(2 /* inputs */, 1 /* size */));
    adder1_->set_name("adder1");

    integrator0_.reset(new Integrator<double>(1 /* size */));
    integrator1_.reset(new Integrator<double>(1 /* size */));

    context_.reset(new DiagramContext<double>(kNumSystems));
    context_->set_time(kTime);

    AddSystem(*adder0_, 0);
    AddSystem(*adder1_, 1);
    AddSystem(*integrator0_, 2);
    AddSystem(*integrator1_, 3);

    context_->ExportInput({0 /* adder0_ */, 1 /* port 1 */});
    context_->ExportInput({1 /* adder1_ */, 0 /* port 0 */});

    context_->MakeState();
    ContinuousState<double>* xc = context_->get_mutable_continuous_state();
    xc->get_mutable_vector()->SetAtIndex(0, 42.0);
    xc->get_mutable_vector()->SetAtIndex(1, 43.0);
  }

  void AddSystem(const System<double>& sys, int index) {
    auto subcontext = sys.CreateDefaultContext();
    auto suboutput = sys.AllocateOutput(*subcontext);
    context_->AddSystem(index, std::move(subcontext), std::move(suboutput));
  }

  void AttachInputPorts() {
    auto vec0 = std::make_unique<BasicVector<double>>(1 /* size */);
    vec0->get_mutable_value() << 128;
    auto vec1 = std::make_unique<BasicVector<double>>(1 /* size */);
    vec1->get_mutable_value() << 256;

    context_->SetInputPort(
        0, std::make_unique<FreestandingInputPort>(std::move(vec0)));
    context_->SetInputPort(
        1, std::make_unique<FreestandingInputPort>(std::move(vec1)));
  }

  // Mocks up a descriptor that's sufficient to read a FreestandingInputPort
  // connected to @p context at @p index.
  static const BasicVector<double>* ReadVectorInputPort(
      const Context<double>& context, int index) {
    SystemPortDescriptor<double> descriptor(
        nullptr, kInputPort, index, kVectorValued, 0, kInheritedSampling);
    return context.EvalVectorInput(nullptr, descriptor);
  }

  std::unique_ptr<DiagramContext<double>> context_;
  std::unique_ptr<Adder<double>> adder0_;
  std::unique_ptr<Adder<double>> adder1_;
  std::unique_ptr<Integrator<double>> integrator0_;
  std::unique_ptr<Integrator<double>> integrator1_;
};

// Tests that subsystems have outputs and contexts in the DiagramContext.
TEST_F(DiagramContextTest, RetrieveConstituents) {
  // All of the subsystems should be leaf Systems.
  for (int i = 0; i < kNumSystems; ++i) {
    auto context = dynamic_cast<const LeafContext<double>*>(
        context_->GetSubsystemContext(i));
    EXPECT_TRUE(context != nullptr);

    auto output = dynamic_cast<const LeafSystemOutput<double>*>(
        context_->GetSubsystemOutput(i));
    EXPECT_TRUE(output != nullptr);
  }
}

// Tests that the time writes through to the subsystem contexts.
TEST_F(DiagramContextTest, Time) {
  context_->set_time(42.0);
  EXPECT_EQ(42.0, context_->get_time());
  for (int i = 0; i < kNumSystems; ++i) {
    EXPECT_EQ(42.0, context_->GetSubsystemContext(i)->get_time());
  }
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(DiagramContextTest, State) {
  ContinuousState<double>* xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(2, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());

  // Changes to the diagram state write through to constituent system states.
  ContinuousState<double>* integrator0_xc =
      context_->GetMutableSubsystemContext(2)->get_mutable_continuous_state();
  ContinuousState<double>* integrator1_xc =
      context_->GetMutableSubsystemContext(3)->get_mutable_continuous_state();
  EXPECT_EQ(42.0, integrator0_xc->get_vector().GetAtIndex(0));
  EXPECT_EQ(43.0, integrator1_xc->get_vector().GetAtIndex(0));

  // Changes to constituent system states appear in the diagram state.
  integrator1_xc->get_mutable_vector()->SetAtIndex(0, 1000.0);
  EXPECT_EQ(1000.0, xc->get_vector().GetAtIndex(1));
}

// Tests that no exception is thrown when connecting a valid source
// and destination port.
TEST_F(DiagramContextTest, ConnectValid) {
  EXPECT_NO_THROW(context_->Connect({0 /* adder0_ */, 0 /* port 0 */},
                                    {1 /* adder1_ */, 1 /* port 1 */}));
}

// Tests that input ports can be assigned to the DiagramContext and then
// retrieved.
TEST_F(DiagramContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(2, context_->get_num_input_ports());
  AttachInputPorts();
  EXPECT_EQ(128, ReadVectorInputPort(*context_, 0)->get_value()[0]);
  EXPECT_EQ(256, ReadVectorInputPort(*context_, 1)->get_value()[0]);
}

TEST_F(DiagramContextTest, Clone) {
  context_->Connect({0 /* adder0_ */, 0 /* port 0 */},
                    {1 /* adder1_ */, 1 /* port 1 */});
  AttachInputPorts();

  std::unique_ptr<DiagramContext<double>> clone(
      dynamic_cast<DiagramContext<double>*>(context_->Clone().release()));
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state was copied.
  const ContinuousState<double>* xc = context_->get_continuous_state();

  EXPECT_EQ(42.0, xc->get_vector().GetAtIndex(0));
  EXPECT_EQ(43.0, xc->get_vector().GetAtIndex(1));

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(2, clone->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const BasicVector<double>* orig_port = ReadVectorInputPort(*context_, i);
    const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, i);
    EXPECT_NE(orig_port, clone_port);
    EXPECT_TRUE(CompareMatrices(orig_port->get_value(), clone_port->get_value(),
                                1e-8, MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

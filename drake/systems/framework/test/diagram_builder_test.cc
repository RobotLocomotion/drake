#include "drake/systems/framework/diagram_builder.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace {
// Tests that an exception is thrown if the diagram contains an algebraic loop.
GTEST_TEST(DiagramBuilderTest, AlgebraicLoop) {
  DiagramBuilder<double> builder;
  auto adder = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  // Connect the output port to the input port.
  builder.Connect(adder->get_output_port(), adder->get_input_port(0));
  EXPECT_THROW(builder.Build(), std::logic_error);
}

// Tests that a cycle which is not an algebraic loop, because one of the
// components is not direct-feedthrough, can be resolved.
GTEST_TEST(DiagramBuilderTest, CycleButNoAlgebraicLoop) {
  DiagramBuilder<double> builder;

  // Create the following diagram:
  //
  // input --->| 0       |
  //           |   Adder +---> Integrator -|
  //        |->| 1       |                 |---> output
  //        |------------------------------|
  auto adder = builder.AddSystem<Adder>(2 /* inputs */, 1 /* size */);
  auto integrator = builder.AddSystem<Integrator>(1 /* size */);

  builder.Connect(integrator->get_output_port(0), adder->get_input_port(1));
  builder.ExportInput(adder->get_input_port(0));
  builder.ExportOutput(integrator->get_output_port(0));

  // There is no algebraic loop, so we should not throw.
  EXPECT_NO_THROW(builder.Build());
}

// Tests that multiple cascaded elements that are not direct-feedthrough
// are sortable.
GTEST_TEST(DiagramBuilderTest, CascadedNonDirectFeedthrough) {
  DiagramBuilder<double> builder;

  auto integrator1 = builder.AddSystem<Integrator>(1 /* size */);
  auto integrator2 = builder.AddSystem<Integrator>(1 /* size */);

  builder.Connect(integrator1->get_output_port(0),
                  integrator2->get_input_port(0));
  builder.ExportInput(integrator1->get_input_port(0));
  builder.ExportOutput(integrator2->get_output_port(0));

  // There is no algebraic loop, so we should not throw.
  EXPECT_NO_THROW(builder.Build());
}

// Tests that an exception is thrown when building an empty diagram.
GTEST_TEST(DiagramBuilderTest, FinalizeWhenEmpty) {
  DiagramBuilder<double> builder;
  EXPECT_THROW(builder.Build(), std::logic_error);
}

GTEST_TEST(DiagramBuilderTest, SystemsThatAreNotAddedThrow) {
  DiagramBuilder<double> builder;
  Adder<double> adder(1 /* inputs */, 1 /* size */);
  EXPECT_THROW(builder.Connect(adder, adder), std::exception);
  EXPECT_THROW(builder.ExportInput(adder.get_input_port(0)), std::exception);
  EXPECT_THROW(builder.ExportOutput(adder.get_output_port()), std::exception);
}

// Helper class that has one input port, and no output ports.
template <typename T>
class Sink : public LeafSystem<T> {
 public:
  Sink() { this->DeclareInputPort(kVectorValued, 1, kContinuousSampling); }
  void EvalOutput(const Context<T>&, SystemOutput<T>*) const override {}
};

// Tests the sole-port based overload of Connect().
class DiagramBuilderSolePortsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    out1_ = builder_.AddSystem<ConstantVectorSource>(Vector1d::Ones());
    in1_ = builder_.AddSystem<Sink>();
    in1out1_ = builder_.AddSystem<Gain>(1.0 /* gain */, 1 /* size */);
    in2out1_ = builder_.AddSystem<Adder>(2 /* inputs */, 1 /* size */);
    in1out2_ = builder_.AddSystem<Demultiplexer>(2 /* size */);
  }

  DiagramBuilder<double> builder_;
  ConstantVectorSource<double>* out1_ = nullptr;
  Sink<double>* in1_ = nullptr;
  Gain<double>* in1out1_ = nullptr;
  Adder<double>* in2out1_ = nullptr;
  Demultiplexer<double>* in1out2_ = nullptr;
};

// A diagram of Source->Gain->Sink is successful.
TEST_F(DiagramBuilderSolePortsTest, SourceGainSink) {
  EXPECT_NO_THROW(builder_.Connect(*out1_, *in1out1_));
  EXPECT_NO_THROW(builder_.Connect(*in1out1_, *in1_));
  EXPECT_NO_THROW(builder_.Build());
}

// The cascade synonym also works.
TEST_F(DiagramBuilderSolePortsTest, SourceGainSinkCascade) {
  EXPECT_NO_THROW(builder_.Cascade(*out1_, *in1out1_));
  EXPECT_NO_THROW(builder_.Cascade(*in1out1_, *in1_));
  EXPECT_NO_THROW(builder_.Build());
}

// A diagram of Gain->Source is has too few dest inputs.
TEST_F(DiagramBuilderSolePortsTest, TooFewDestInputs) {
  EXPECT_THROW(builder_.Connect(*in1out1_, *out1_), std::exception);
}

// A diagram of Source->In2out1 is has too many dest inputs.
TEST_F(DiagramBuilderSolePortsTest, TooManyDestInputs) {
  using std::exception;
  EXPECT_THROW(builder_.Connect(*out1_, *in2out1_), std::exception);
}

// A diagram of Sink->Gain is has too few src inputs.
TEST_F(DiagramBuilderSolePortsTest, TooFewSrcInputs) {
  using std::exception;
  EXPECT_THROW(builder_.Connect(*in1_, *in1out1_), std::exception);
}

// A diagram of Demux->Gain is has too many src inputs.
TEST_F(DiagramBuilderSolePortsTest, TooManySrcInputs) {
  using std::exception;
  EXPECT_THROW(builder_.Connect(*in1out2_, *in1out1_), std::exception);
}

// Test for GetMutableSystems.
GTEST_TEST(DiagramBuilderTest, GetMutableSystems) {
  DiagramBuilder<double> builder;
  auto adder1 = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  auto adder2 = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  EXPECT_EQ((std::vector<System<double>*>{adder1, adder2}),
            builder.GetMutableSystems());
}

}  // namespace
}  // namespace systems
}  // namespace drake

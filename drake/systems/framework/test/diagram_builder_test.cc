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
  Adder<double> adder(1 /* inputs */, 1 /* length */);
  // Connect the output port to the input port.
  builder.Connect(adder.get_output_port(0), adder.get_input_port(0));
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
  Adder<double> adder(2 /* inputs */, 1 /* length */);
  Integrator<double> integrator(1 /* length */);

  builder.Connect(integrator.get_output_port(0), adder.get_input_port(1));
  builder.ExportInput(adder.get_input_port(0));
  builder.ExportOutput(integrator.get_output_port(0));

  // There is no algebraic loop, so we should not throw.
  EXPECT_NO_THROW(builder.Build());
}

// Tests that multiple cascaded elements that are not direct-feedthrough
// are sortable.
GTEST_TEST(DiagramBuilderTest, CascadedNonDirectFeedthrough) {
  DiagramBuilder<double> builder;

  Integrator<double> integrator1(1 /* length */);
  Integrator<double> integrator2(1 /* length */);

  builder.Connect(integrator1.get_output_port(0),
                  integrator2.get_input_port(0));
  builder.ExportInput(integrator1.get_input_port(0));
  builder.ExportOutput(integrator2.get_output_port(0));

  // There is no algebraic loop, so we should not throw.
  EXPECT_NO_THROW(builder.Build());
}

// Tests that an exception is thrown when building an empty diagram.
GTEST_TEST(DiagramBuilderTest, FinalizeWhenEmpty) {
  DiagramBuilder<double> builder;
  EXPECT_THROW(builder.Build(), std::logic_error);
}

// Helper class that has one input port, and no output ports.
template <typename T>
class Sink : public LeafSystem<T> {
 public:
  Sink() { this->DeclareInputPort(kVectorValued, 1, kContinuousSampling); }
  void EvalOutput(const Context<T>&, SystemOutput<T>*) const override {}
};

// Tests the sole-port based overload of Connect().
GTEST_TEST(DiagramBuilderTest, ConnectSolePorts) {
  const ConstantVectorSource<double> out1(Vector1d::Ones());
  const Sink<double> in1;
  const Gain<double> in1out1(1.0 /* gain */, 1 /* length */);
  const Adder<double> in2out1(2 /* inputs */, 1 /* length */);
  const Demultiplexer<double> in1out2(2 /* length */);

  // A diagram of Source->Gain->Sink is successful.
  {
    DiagramBuilder<double> builder;
    EXPECT_NO_THROW(builder.Connect(out1, in1out1));
    EXPECT_NO_THROW(builder.Connect(in1out1, in1));
    EXPECT_NO_THROW(builder.Build());
  }

  // The cascade synonym also works.
  EXPECT_NO_THROW(DiagramBuilder<double>().Cascade(out1, in1out1));

  // A diagram of Gain->Source is has too few dest inputs.
  using std::exception;
  EXPECT_THROW(DiagramBuilder<double>().Connect(in1out1, out1), exception);

  // A diagram of Source->In2out1 is has too many dest inputs.
  EXPECT_THROW(DiagramBuilder<double>().Connect(out1, in2out1), exception);

  // A diagram of Sink->Gain is has too few src inputs.
  EXPECT_THROW(DiagramBuilder<double>().Connect(in1, in1out1), exception);

  // A diagram of Demux->Gain is has too many src inputs.
  EXPECT_THROW(DiagramBuilder<double>().Connect(in1out2, in1out1), exception);
}

}  // namespace
}  // namespace systems
}  // namespace drake

#include "drake/systems/framework/diagram_builder.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/adder.h"
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

}  // namespace
}  // namespace systems
}  // namespace drake

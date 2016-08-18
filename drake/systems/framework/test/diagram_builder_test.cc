#include "drake/systems/framework/diagram_builder.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace {

// Tests that an exception is thrown if the diagram contains a cycle.
GTEST_TEST(DiagramBuilderTest, Cycle) {
  DiagramBuilder<double> builder;
  Adder<double> adder(1 /* inputs */, 1 /* length */);
  // Connect the output port to the input port.
  builder.Connect(adder.get_output_port(0), adder.get_input_port(0));
  EXPECT_THROW(builder.Build(), std::logic_error);
}

// Tests that an exception is thrown when building an empty diagram.
GTEST_TEST(DiagramBuilderTest, FinalizeWhenEmpty) {
  DiagramBuilder<double> builder;
  EXPECT_THROW(builder.Build(), std::logic_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake

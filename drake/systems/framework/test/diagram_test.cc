#include "drake/systems/framework/diagram.h"

#include "drake/systems/framework/primitives/adder.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace drake {
namespace systems {
namespace {

/// Tests the following diagram:
/// adder1: (input[0] + input[1]) -> A
/// adder2: (A + input[2])        -> B, output[0]
/// adder3: (A + B)               -> output[1]
GTEST_TEST(DiagramTest, SystemOfAdders) {
  Diagram<double> diagram("Unicode Snowman's Favorite Diagram!!1!☃!");
  Adder<double> adder1(2 /* inputs */, 3 /* length */);
  Adder<double> adder2(2 /* inputs */, 3 /* length */);
  Adder<double> adder3(2 /* inputs */, 3 /* length */);

  diagram.Connect(&adder1, 0 /* src_port_index */,
                  &adder2, 0 /* dest_port_index */);
  diagram.Connect(&adder1, 0 /* src_port_index */,
                  &adder3, 0 /* dest_port_index */);
  diagram.Connect(&adder2, 0 /* src_port_index */,
                  &adder3, 1 /* dest_port_index */);

  diagram.AddInput(&adder1, 0);
  diagram.AddInput(&adder1, 1);
  diagram.AddInput(&adder2, 1);
  diagram.AddOutput(&adder2, 0);
  diagram.AddOutput(&adder2, 0);

  diagram.Finalize();
  std::unique_ptr<Context<double>> context = diagram.CreateDefaultContext();
}

}  // namespace
}  // namespace systems
}  // namespace drake

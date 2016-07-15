#include "drake/systems/framework/primitives/adder3.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context3.h"
#include "drake/systems/framework/primitives/cascade3.h"
#include "drake/systems/framework/primitives/diagram3.h"
#include "drake/systems/framework/primitives/gain3.h"
#include "drake/systems/framework/primitives/vector_constant3.h"
#include "drake/systems/framework/system3.h"
#include "drake/systems/framework/system3_input.h"

#include "gtest/gtest.h"

using std::unique_ptr;
using std::make_unique;
using std::move;

namespace drake {
namespace systems {
namespace {

GTEST_TEST(AdderTest, AddTwoVectors) {
  Eigen::Vector3d input0;
  input0 << 1, 2, 3;
  Eigen::Vector3d input1;
  input1 << 4, 5, 6;

  // Make diagram like this:
  //  --------------- DIAGRAM ------------
  // |  ----- CASCADE -----     -------   |
  // | | input0 -> gain*2 -|-->|       |  |
  // |  -------------------    | adder |--|-> 2*in0 + in1
  // |                         |       |  |
  // |                input1-->|       |  |
  // |                          -------   |
  //  ------------------------------------
  //
  Eigen::Vector3d expected = 2 * input0 + input1;
  // 6, 9, 12

  Diagram3<double> diagram("AddTwoVectors");
  // Top level has three subsystems, the adder, the cascade, and input1.
  diagram.AddSubsystem(
      make_unique<Adder3<double>>("Adder23", 2 /*inputs*/, 3 /*length*/));
  diagram.AddSubsystem(make_unique<Cascade3<double>>(
      "input0x2", make_unique<VectorConstant3<double>>("input0", input0),
      make_unique<Gain3<double>>("times2", 2., 3 /*length*/)));
  diagram.AddSubsystem(make_unique<VectorConstant3<double>>("input1", input1));

  diagram.Connect(1, 0, 0, 0);
  diagram.Connect(2, 0, 0, 1);
  diagram.InheritOutputPort(0, 0);

  auto context = diagram.CreateDefaultContext();

  // Diagram has no input ports and one output port and context should match.
  ASSERT_EQ(0, context->get_num_input_ports());
  ASSERT_EQ(1, context->get_num_output_ports());

  const auto& output = diagram.EvalVectorOutputPort(*context, 0);

  EXPECT_EQ(expected, output.get_value());
}

// Tests that std::out_of_range is thrown when input ports of the wrong size
// are connected.
GTEST_TEST(AdderTest, WrongSizeOfInputPorts) {
  Eigen::Vector3d input0;
  input0 << 1, 2, 3;
  Eigen::Vector4d input1; // wrong size
  input1 << 4, 5, 6, 7;

  Diagram3<double> diagram("AddTwoVectors");
  // Top level has three subsystems, the adder, the input0, and input1.
  diagram.AddSubsystem(
      make_unique<Adder3<double>>("Adder23", 2 /*inputs*/, 3 /*length*/));
  diagram.AddSubsystem(make_unique<VectorConstant3<double>>("input0", input0));
  diagram.AddSubsystem(make_unique<VectorConstant3<double>>("input1", input1));

  diagram.Connect(1, 0, 0, 0);
  EXPECT_THROW(diagram.Connect(2, 0, 0, 1), std::out_of_range);
}

// Tests that Adder allocates no state variables in the context_.
GTEST_TEST(AdderTest, AdderIsStateless) {
  /// TODO(david-german-tri): Once state exists in Context, assert it's empty.
}

}  // namespace
}  // namespace systems
}  // namespace drake

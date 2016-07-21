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
  Eigen::Vector3d vec0;
  vec0 << 1, 2, 3;
  Eigen::Vector3d vec1;
  vec1 << 4, 5, 6;

  // Make diagram like this:
  //  --------------- DIAGRAM ------------
  // |   ---- CASCADE -----     -------   |
  // |   | vec0 -> gain*2 -|-->|       |  |
  // |   ------------------    | adder |--|-> 2*vec0 + vec1
  // |                         |       |  |
  // |                  vec1-->|       |  |
  // |                          -------   |
  //  ------------------------------------

  Eigen::Vector3d expected = 2 * vec0 + vec1;
  // 6, 9, 12

  Diagram3<double> diagram("AddTwoVectors");

  // Top level has three subsystems, the adder, the cascade, and vec1.
  // AddSubsystem returns raw pointers of concrete type to the contained
  // subsystem objects in the diagram.
  auto adder = diagram.AddSubsystem(
      make_unique<Adder3<double>>("Adder23", 2 /*inputs*/, 3 /*length*/));
  auto cascade = diagram.AddSubsystem(
      make_unique<Cascade3<double>>("vec0x2", 
          make_unique<VectorConstant3<double>>("vec0", vec0),
          make_unique<Gain3<double>>("times2", 2.0, 3 /*length*/)));
  auto constant1 = diagram.AddSubsystem(
      make_unique<VectorConstant3<double>>("vec1", vec1));

  diagram.Connect(cascade, 0, adder, 0);
  diagram.Connect(constant1, 0, adder, 1);
  diagram.InheritOutputPort(adder, 0);

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
  Eigen::Vector3d vec0;
  vec0 << 1, 2, 3;
  Eigen::Vector4d vec1;  // Wrong size.
  vec1 << 4, 5, 6, 7;

  Diagram3<double> diagram("AddTwoVectors");
  //  ------ DIAGRAM -------
  // |            -------   |
  // |    vec0-->|       |  |
  // |           | adder |--|-> vec0 + vec1
  // |           |       |  |
  // |    vec1-->|       |  |
  // |            -------   |
  //  ----------------------

  auto adder = diagram.AddSubsystem(
      make_unique<Adder3<double>>("Adder23", 2 /*inputs*/, 3 /*length*/));
  auto constant0 = diagram.AddSubsystem(
      make_unique<VectorConstant3<double>>("vec0", vec0));
  auto constant1 = diagram.AddSubsystem(
      make_unique<VectorConstant3<double>>("vec1", vec1));

  diagram.Connect(constant0, 0, adder, 0);
  EXPECT_THROW(diagram.Connect(constant1, 0, adder, 1), std::out_of_range);
}

// Tests that Adder allocates no state variables in the context_.
GTEST_TEST(AdderTest, AdderIsStateless) {
  /// TODO(david-german-tri): Once state exists in Context, assert it's empty.
}

}  // namespace
}  // namespace systems
}  // namespace drake

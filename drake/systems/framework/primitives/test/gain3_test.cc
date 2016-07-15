#include "drake/systems/framework/primitives/gain3.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context3.h"
#include "drake/systems/framework/system3.h"
#include "drake/systems/framework/system3_input.h"
#include "drake/systems/framework/primitives/diagram3.h"
#include "drake/systems/framework/primitives/vector_constant3.h"

#include "gtest/gtest.h"

using std::unique_ptr;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

GTEST_TEST(GainTest, MultiplyVectorBy3) {
  Eigen::Vector3d input;
  input << 1, 2, 3;
  Diagram3<double> diagram("multiplyBy3"); // could have used Cascade3 here.
  diagram.AddSubsystem(make_unique<VectorConstant3<double>>("v123", input));
  diagram.AddSubsystem(make_unique<Gain3<double>>("gain3", 3., 3 /*length*/));

  diagram.Connect(0, 0, 1, 0);
  diagram.InheritOutputPort(1, 0);

  auto context = diagram.CreateDefaultContext();

  // Diagram has no input ports and one output port and context should match.
  ASSERT_EQ(0, context->get_num_input_ports());
  ASSERT_EQ(1, context->get_num_output_ports());

  const auto& result = diagram.EvalVectorOutputPort(*context, 0);

  Eigen::Vector3d expected;
  expected << 3, 6, 9;
  EXPECT_EQ(expected, result.get_value());
}

}  // namespace
}  // namespace systems
}  // namespace drake

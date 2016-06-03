#include "drake/systems/framework/context.h"

#include <memory>
#include <stdexcept>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {

GTEST_TEST(ContextTest, GetNumInputPorts) {
  Context<int> context;
  ASSERT_EQ(0, context.get_num_input_ports());
  context.get_mutable_input()->ports.resize(2);
  EXPECT_EQ(2, context.get_num_input_ports());
}

GTEST_TEST(ContextTest, GetVectorInput) {
  Context<int> context;
  context.get_mutable_input()->ports.resize(2);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
  vec->get_mutable_value() << 5, 6;
  std::unique_ptr<FreestandingInputPort<int>> port(
      new FreestandingInputPort<int>(std::move(vec), 0.0 /* continuous */));
  context.get_mutable_input()->ports[0] = std::move(port);

  // Test that port 0 is retrievable.
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, context.get_vector_input(0)->get_value());

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, context.get_vector_input(1));

  // Test that out-of-bounds ports throw an exception.
  EXPECT_THROW(context.get_vector_input(2), std::out_of_range);
}

}  // namespace systems
}  // namespace drake

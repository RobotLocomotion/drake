#include "drake/systems/primitives/pass_through-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include <memory>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector3d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

// Tests the ability to take derivatives of the output with respect to
// the input. Since `y = u` the derivative in this case is the identity matrix.
GTEST_TEST(PassThroughScalarTypeTest, AutoDiff) {
  // In this unit test with vectors of size three, derivatives will be taken
  // with respect to the entire input vector. Therefore the Vector3d template
  // argument.
  typedef AutoDiffScalar<Vector3d> T;

  // Set a PassThrough system with input and output of size 3.
  auto buffer = make_unique<PassThrough<T>>(3 /* size */);
  auto context = buffer->CreateDefaultContext();
  auto output = buffer->AllocateOutput(*context);
  auto input = make_unique<BasicVector<T>>(3 /* size */);

  // Sets the input values.
  Vector3<T> input_vector(1.0, 3.14, 2.18);

  // Sets the input vector to be the vector of independent variables.
  input_vector(0).derivatives() << 1, 0, 0;  // First independent variable.
  input_vector(1).derivatives() << 0, 1, 0;  // Second independent variable.
  input_vector(2).derivatives() << 0, 0, 1;  // Third independent variable.

  input->get_mutable_value() << input_vector;

  context->FixInputPort(0, std::move(input));

  buffer->CalcOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  auto output_vector = output->get_vector_data(0)->get_value();

  // The expected output value equals the input.
  Vector3<T> expected;
  expected = input_vector;

  // The expected derivative is the identity matrix:
  expected(0).derivatives() << 1.0, 0.0, 0.0;
  expected(1).derivatives() << 0.0, 1.0, 0.0;
  expected(2).derivatives() << 0.0, 0.0, 1.0;

  for (int i = 0; i < 3; ++i) {
    // Checks output value.
    EXPECT_EQ(expected(i).value(), output_vector(i).value());

    // Checks derivatives.
    EXPECT_EQ(expected(i).derivatives(), output_vector(i).derivatives());
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

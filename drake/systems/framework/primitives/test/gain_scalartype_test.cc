#include "drake/systems/framework/primitives/gain-inl.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the Gain system.
template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

// Tests the ability to take derivatives of the output with respect to some
// (not all) of the inputs.
// In this unit test derivatives are taken with respect to the first and third
// input variables. The second input is set to be a constant and therefore
// derivatives with respect to this input are zero.
GTEST_TEST(GainScalarTypeTest, AutoDiff) {
  // There are only two independent variables in this problem with respect to
  // which we want to take derivatives. Therefore Vector2d_unaligned is the
  // template argument of AutoDiffScalar.
  // TODO(amcastro-tri): change to Vector2d once #3145 is fixed.
  typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2d_unaligned;
  typedef AutoDiffScalar<Vector2d_unaligned> T;

  // Set a Gain system with input and output of size 3. Notice that this size
  // does not necessarily need to be the same as the size of the derivatives
  // vectors and, for this unit test, they are not equal.
  const double kGain = 2.0;
  auto gain = make_unique<Gain<T>>(kGain /* gain */, 3 /* size */);
  auto context = gain->CreateDefaultContext();
  auto output = gain->AllocateOutput(*context);
  auto input = make_unique<BasicVector<T>>(3 /* size */);

  // Sets the input values.
  VectorX<T> input_vector(3);
  input_vector << 1.0, 3.14, 2.18;

  // Sets the independent variables to be the first and third input entries.
  input_vector(0).derivatives() << 1, 0;  // First independent variable.
  input_vector(1).derivatives() << 0, 0;  // Constant input.
  input_vector(2).derivatives() << 0, 1;  // Second independent variable.

  input->get_mutable_value() << input_vector;

  context->SetInputPort(0, MakeInput(std::move(input)));

  gain->EvalOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const auto& output_vector = output->get_vector_data(0)->get_value();

  // The expected output value is the gain times the input vector.
  VectorX<T> expected = kGain * input_vector;

  // The expected derivatives are:
  expected(0).derivatives() << kGain, 0.0;
  expected(1).derivatives() << 0.0, 0.0;
  expected(2).derivatives() << 0.0, kGain;

  const double tolerance = Eigen::NumTraits<double>::epsilon();
  for (int i = 0; i < 3; ++i) {
    // Checks output value.
    EXPECT_NEAR(expected(i).value(), output_vector(i).value(), tolerance);

    // Checks derivatives.
    EXPECT_TRUE(expected(i).derivatives().isApprox(
        output_vector(i).derivatives(), tolerance));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

#include "drake/systems/framework/primitives/buffer-inl.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <unsupported/Eigen/AutoDiff>

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
// the input of the PassThrough system.
template<class T>
std::unique_ptr<FreestandingInputPort<T>> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort<T>>(std::move(data));
}

class BufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    buffer_ = make_unique<PassThrough<double>>(3 /* length */);
    context_ = buffer_->CreateDefaultContext();
    output_ = buffer_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  std::unique_ptr<PassThrough<double>> buffer_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(BufferTest, VectorThroughBufferSystem) {
  // Hook input of the expected size.
  // TODO(amcastro-tri): we must be able to ask buffer_->num_of_input_ports().
  ASSERT_EQ(1, context_->get_num_input_ports());
  Eigen::Vector3d input_vector(1.0, 3.14, 2.18);
  input_->get_mutable_value() << input_vector;

  context_->SetInputPort(0, MakeInput(std::move(input_)));

  buffer_->EvalOutput(*context_, output_.get());

  // TODO(amcastro-tri): we must be able to ask buffer_->num_of_output_ports().
  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_vector =
      dynamic_cast<const BasicVector<double>*>(
          output_->get_port(0).get_vector_data());
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(input_vector, output_vector->get_value());
}

// Tests that std::out_of_range is thrown when the wrong number of input ports
// are connected.
TEST_F(BufferTest, NoInputPorts) {
  // No input ports are hooked up. PassThrough must have one input port.
  // TODO(amcastro-tri): This will not be needed here when input/outputs are
  // defined in the constructor.
  // Connections sanity check will be performed by Diagram::Finalize().

  // TODO(amcastro-tri): we must be able to ask buffer_->num_of_input_ports()
  // and make the GTest with that.
  EXPECT_THROW(buffer_->EvalOutput(*context_, output_.get()),
               std::out_of_range);
}

// Tests that std::out_of_range is thrown when input ports of the wrong size
// are connected.
// TODO(amcastro-tri): when #3109 is resolved verify that input and output ports
// are the same size even if their sizes were determined automatically.
TEST_F(BufferTest, WrongSizeOfInputPorts) {
  // Hook up input port, but of the wrong size.
  // TODO(amcastro-tri): we must be able to ask buffer_->num_of_input_ports().
  ASSERT_EQ(1, context_->get_num_input_ports());
  auto short_input = make_unique<BasicVector<double>>(2 /* length */);
  short_input->get_mutable_value() << 4, 5;
  context_->SetInputPort(0, MakeInput(std::move(short_input)));

  EXPECT_THROW(buffer_->EvalOutput(*context_, output_.get()),
               std::out_of_range);
}

// Tests that PassThrough allocates no state variables in the context_.
TEST_F(BufferTest, BufferIsStateless) {
  EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}

// Tests the capability to take derivatives of the output with respect to
// the input. Since `y = u` the derivative in this case is the identity matrix.
GTEST_TEST(MiscBufferTests, AutoDiff) {
  // In this unit test with vectors of length three, derivatives will be taken
  // with rexpect to the entire input vector. Therefore the Vector3d template
  // argument.
  typedef AutoDiffScalar<Vector3d> T;

  // Set a PassThrough system with input and output of size 3.
  auto buffer = make_unique<PassThrough<T>>(3 /* length */);
  auto context = buffer->CreateDefaultContext();
  auto output = buffer->AllocateOutput(*context);
  auto input = make_unique<BasicVector<T>>(3 /* length */);

  // Sets the input values.
  Vector3<T> input_vector(1.0, 3.14, 2.18);

  // Sets the independent variables to be the first and third input entries.
  input_vector(0).derivatives() << 1, 0, 0;  // First independent variable.
  input_vector(1).derivatives() << 0, 1, 0;  // Second independent variable.
  input_vector(2).derivatives() << 0, 0, 1;  // Third independent variable.

  input->get_mutable_value() << input_vector;

  context->SetInputPort(0, MakeInput(std::move(input)));

  buffer->EvalOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const auto& output_vector =
      dynamic_cast<const BasicVector<T> *>(
          output->get_port(0).get_vector_data())->get_value();

  // The expected output value equals the input.
  Vector3<T> expected;
  expected = input_vector;

  // The expected derivative is the identity matrix:
  expected(0).derivatives() << 1.0, 0.0, 0.0;
  expected(1).derivatives() << 0.0, 1.0, 0.0;
  expected(2).derivatives() << 0.0, 0.0, 1.0;

  const double tolerance = Eigen::NumTraits<double>::epsilon();
  for (int i=0; i < 3; i++) {
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

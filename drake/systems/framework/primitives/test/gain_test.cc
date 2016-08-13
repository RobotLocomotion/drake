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
template<class T>
std::unique_ptr<FreestandingInputPort<T>> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort<T>>(std::move(data));
}

class GainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gain_ = make_unique<Gain<double>>(kGain_ /* gain */, 3 /* length */);
    context_ = gain_->CreateDefaultContext();
    output_ = gain_->AllocateOutput(*context_);
    input0_ = make_unique<BasicVector<double>>(3 /* length */);
    input1_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  const double kGain_{2.0};
  std::unique_ptr<Gain<double>> gain_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
};

TEST_F(GainTest, VectorThroughGainSystem) {
  // Hook input of the expected size.
  // TODO(amcastro-tri): we must be able to ask gain_->num_of_input_ports().
  ASSERT_EQ(1, context_->get_num_input_ports());
  Eigen::Vector3d input_vector(1.0, 3.14, 2.18);
  input0_->get_mutable_value() << input_vector;

  context_->SetInputPort(0, MakeInput(std::move(input0_)));

  gain_->EvalOutput(*context_, output_.get());

  // TODO(amcastro-tri): we must be able to ask gain_->num_of_output_ports().
  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_vector =
      dynamic_cast<const BasicVector<double>*>(
          output_->get_port(0).get_vector_data());
  ASSERT_NE(nullptr, output_vector);
  Eigen::Vector3d expected = kGain_ * input_vector;
  EXPECT_EQ(expected, output_vector->get_value());
}

// Tests that std::out_of_range is thrown when the wrong number of input ports
// are connected.
TEST_F(GainTest, NoInputPorts) {
  // No input ports are hooked up. Gain must have one input port.
  // TODO(amcastro-tri): This will not be needed here when input/outputs are
  // defined in the constructor.
  // Connections sanity check will be performed by Diagram::Finalize().

  // TODO(amcastro-tri): we must be able to ask gain_->num_of_input_ports()
  // and make the GTest with that.
  EXPECT_THROW(gain_->EvalOutput(*context_, output_.get()), std::out_of_range);
}

// Tests that std::out_of_range is thrown when input ports of the wrong size
// are connected.
// TODO(amcastro-tri): when #3109 is resolved verify that input and output ports
// are the same size even if their sizes were determined automatically.
TEST_F(GainTest, WrongSizeOfInputPorts) {
  // Hook up input port, but of the wrong size.
  // TODO(amcastro-tri): we must be able to ask gain_->num_of_input_ports().
  ASSERT_EQ(1, context_->get_num_input_ports());
  auto short_input = make_unique<BasicVector<double>>(2 /* length */);
  short_input->get_mutable_value() << 4, 5;
  context_->SetInputPort(0, MakeInput(std::move(short_input)));

  EXPECT_THROW(gain_->EvalOutput(*context_, output_.get()), std::out_of_range);
}

// Tests that Gain allocates no state variables in the context_.
TEST_F(GainTest, GainIsStateless) {
  EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}

// Tests the ability to take derivatives of the output with respect to some
// (not all) of the inputs.
// In this unit test derivatives are taken with respect to the first and third
// input variables. The second input is set to be a constant and therefore
// derivatives with respect to this input are zero.
GTEST_TEST(MiscGainTests, AutoDiff) {
  // There are only two independent variables in this problem with respect to
  // which we want to take derivatives. Therefore the Vector2d in the template
  // argument of AutoDiffScalar.
  typedef AutoDiffScalar<Vector2d> T;

  // Set a Gain system with input and output of size 3. Notice that this size
  // does not necessarily need to be the same as the size of the derivatives
  // vectors and, for this unit test, they are not equal.
  const double kGain = 2.0;
  auto gain = make_unique<Gain<T>>(kGain /* gain */, 3 /* length */);
  auto context = gain->CreateDefaultContext();
  auto output = gain->AllocateOutput(*context);
  auto input = make_unique<BasicVector<T>>(3 /* length */);

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
  const auto& output_vector =
      dynamic_cast<const BasicVector<T> *>(
          output->get_port(0).get_vector_data())->get_value();

  // The expected output value is the gain times the input vector.
  VectorX<T> expected = (kGain * input_vector).eval();

  // The expected derivatives are:
  expected(0).derivatives() << kGain, 0.0;
  expected(1).derivatives() << 0.0, 0.0;
  expected(2).derivatives() << 0.0, kGain;

  const double tolerance = Eigen::NumTraits<double>::epsilon();
  for (int i=0; i < 3; ++i) {
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

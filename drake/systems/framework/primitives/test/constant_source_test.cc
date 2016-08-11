#include "drake/systems/framework/primitives/constant_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::Vector2d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class ConstantSourceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_ = make_unique<ConstantSource<double>>(kConstantSource_);
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
    input0_ = make_unique<BasicVector<double>>(3 /* length */);
    input1_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  // TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
  // the input of the ConstantSource system.
  static std::unique_ptr<FreestandingInputPort<double>> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return make_unique<FreestandingInputPort<double>>(std::move(data));
  }

  const Vector2d kConstantSource_{2.0, 1.5};
  std::unique_ptr<ConstantSource<double>> source_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
};

TEST_F(ConstantSourceTest, OutputTest) {
  // Hook input of the expected size.
  // TODO(amcastro-tri): we must be able to ask source_->num_of_input_ports().
  ASSERT_EQ(0, context_->get_num_input_ports());
  // TODO(amcastro-tri): we must be able to ask source_->num_of_output_ports().
  ASSERT_EQ(1, output_->get_num_ports());

  source_->EvalOutput(*context_, output_.get());

  // TODO(amcastro-tri): we should be able to ask something like:
  // auto& source_->get_output_vector(context, 0);
  // to directly get an Eigen expression.
  const BasicVector<double>* output_vector =
      dynamic_cast<const BasicVector<double>*>(
          output_->get_port(0).get_vector_data());
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(kConstantSource_, output_vector->get_value());
}

#if 0
// Tests that std::out_of_range is thrown when the wrong number of input ports
// are connected.
TEST_F(ConstantSourceTest, NoInputPorts) {
  // No input ports are hooked up. ConstantSource must have one input port.
  // TODO(amcastro-tri): This will not be needed here when input/outputs are
  // defined in the constructor.
  // Connections sanity check will be performed by Diagram::Finalize().

  // TODO(amcastro-tri): we must be able to ask source_->num_of_input_ports()
  // and make the GTest with that.
  EXPECT_THROW(source_->EvalOutput(*context_, output_.get()), std::out_of_range);
}

// Tests that std::out_of_range is thrown when input ports of the wrong size
// are connected.
TEST_F(ConstantSourceTest, WrongSizeOfInputPorts) {
  // Hook up input port, but of the wrong size.
  // TODO(amcastro-tri): we must be able to ask source_->num_of_input_ports().
  ASSERT_EQ(1, context_->get_num_input_ports());
  auto short_input = make_unique<BasicVector<double>>(2 /* length */);
  short_input->get_mutable_value() << 4, 5;
  context_->SetInputPort(0, MakeInput(std::move(short_input)));

  EXPECT_THROW(source_->EvalOutput(*context_, output_.get()), std::out_of_range);
}

// Tests that ConstantSource allocates no state variables in the context_.
TEST_F(ConstantSourceTest, ConstantSourceIsStateless) {
  EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}
#endif

}  // namespace
}  // namespace systems
}  // namespace drake

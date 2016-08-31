#include "drake/systems/framework/primitives/demultiplexer.h"

#include <memory>

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
// the input of the Demultiplexer system.
template<class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class DemultiplexerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    demux_ = make_unique<Demultiplexer<double>>(3 /* length */);
    context_ = demux_->CreateDefaultContext();
    output_ = demux_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  std::unique_ptr<System<double>> demux_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

// Tests that the input signal gets demultiplexed into its individual
// components.
TEST_F(DemultiplexerTest, DemultiplexVector) {
  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context_->get_num_input_ports());
  ASSERT_EQ(1, demux_->get_num_input_ports());
  Eigen::Vector3d input_vector(1.0, 3.14, 2.18);
  input_->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  demux_->EvalOutput(*context_, output_.get());

  // Checks that the number of output ports in the system and in the
  // output are consistent.
  // The number of output ports must match the size of the input vector.
  ASSERT_EQ(input_vector.size(), output_->get_num_ports());
  ASSERT_EQ(input_vector.size(), demux_->get_num_output_ports());

  ASSERT_EQ(1, output_->get_vector_data(0)->size());
  ASSERT_EQ(input_vector[0], output_->get_vector_data(0)->get_value()[0]);

  ASSERT_EQ(1, output_->get_vector_data(1)->size());
  ASSERT_EQ(input_vector[1], output_->get_vector_data(1)->get_value()[0]);

  ASSERT_EQ(1, output_->get_vector_data(2)->size());
  ASSERT_EQ(input_vector[2], output_->get_vector_data(2)->get_value()[0]);
}

// Tests that Demultiplexer allocates no state variables in the context_.
TEST_F(DemultiplexerTest, DemultiplexerIsStateless) {
  EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}

}  // namespace
}  // namespace systems
}  // namespace drake

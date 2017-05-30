#include "drake/systems/primitives/demultiplexer.h"

#include <memory>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class DemultiplexerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    demux_ = make_unique<Demultiplexer<double>>(3 /* size */);
    context_ = demux_->CreateDefaultContext();
    output_ = demux_->get_output_port(0).Allocate(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* size */);
  }

  std::unique_ptr<System<double>> demux_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<AbstractValue> output_;
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
  context_->FixInputPort(0, std::move(input_));

  // The number of output ports must match the size of the input vector.
  ASSERT_EQ(input_vector.size(), demux_->get_num_output_ports());

  for (int i=0; i < input_vector.size(); ++i) {
    demux_->get_output_port(i).Calc(*context_, output_.get());
    const auto& output_vector = output_->GetValueOrThrow<BasicVector<double>>();
    ASSERT_EQ(1, output_vector.size());
    ASSERT_EQ(input_vector[i], output_vector.get_value()[0]);
  }
}

// Tests that the input signal gets demultiplexed into its individual
// components.
GTEST_TEST(OutputSize, SizeDifferentFromOne) {
  const int kInputSize = 10;
  const int kOutputSize = 2;
  const int kNumOutputs = kInputSize / kOutputSize;

  // Creates a demultiplexer with an input port of size ten and output ports of
  // size two. Therefore there are five output ports.
  auto demux = make_unique<Demultiplexer<double>>(
      kInputSize /* size */, kOutputSize /* output_ports_sizes */);
  auto context = demux->CreateDefaultContext();
  auto output = demux->get_output_port(0).Allocate(*context);
  auto input = make_unique<BasicVector<double>>(kInputSize /* size */);

  // Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context->get_num_input_ports());
  ASSERT_EQ(1, demux->get_num_input_ports());
  Eigen::VectorXd input_vector = Eigen::VectorXd::Random(kInputSize);
  input->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context->FixInputPort(0, std::move(input));

  // The number of output ports must equal the size of the input port divided
  // by the size of the output ports provided in the constructor, in this case
  // 10 / 2 = 5.
  ASSERT_EQ(kNumOutputs, demux->get_num_output_ports());

  for (int output_index = 0; output_index < kNumOutputs; ++output_index) {
    demux->get_output_port(output_index).Calc(*context, output.get());
    const auto& output_vector = output->GetValueOrThrow<BasicVector<double>>();
    ASSERT_EQ(kOutputSize, output_vector.size());
    ASSERT_EQ(input_vector.segment<kOutputSize>(output_index * kOutputSize),
              output_vector.get_value());
  }
}

// Tests that Demultiplexer allocates no state variables in the context_.
TEST_F(DemultiplexerTest, DemultiplexerIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

TEST_F(DemultiplexerTest, DirectFeedthrough) {
  EXPECT_TRUE(demux_->HasAnyDirectFeedthrough());
  for (int i = 0; i < demux_->get_num_output_ports(); ++i) {
    EXPECT_TRUE(demux_->HasDirectFeedthrough(0, i));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

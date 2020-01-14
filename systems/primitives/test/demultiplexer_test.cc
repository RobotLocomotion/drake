#include "drake/systems/primitives/demultiplexer.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

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
  }

  std::unique_ptr<System<double>> demux_;
  std::unique_ptr<Context<double>> context_;
};

// Tests that the input signal gets demultiplexed into its individual
// components.
TEST_F(DemultiplexerTest, DemultiplexVector) {
  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context_->num_input_ports());
  ASSERT_EQ(1, demux_->num_input_ports());
  Eigen::Vector3d input_vector(1.0, 3.14, 2.18);

  // Hook input of the expected size.
  demux_->get_input_port(0).FixValue(context_.get(), input_vector);

  // The number of output ports must match the size of the input vector.
  ASSERT_EQ(input_vector.size(), demux_->num_output_ports());

  for (int i = 0; i < input_vector.size(); ++i) {
    const auto& output_vector = demux_->get_output_port(i).Eval(*context_);
    ASSERT_EQ(1, output_vector.size());
    ASSERT_EQ(input_vector[i], output_vector[0]);
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
      kInputSize /* size */, kOutputSize /* output_ports_size */);
  auto context = demux->CreateDefaultContext();

  // Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context->num_input_ports());
  ASSERT_EQ(1, demux->num_input_ports());
  Eigen::VectorXd input_vector =
      Eigen::VectorXd::LinSpaced(kInputSize, 1.0, 6.0);

  // Hook input of the expected size.
  demux->get_input_port(0).FixValue(context.get(), input_vector);

  // The number of output ports must equal the size of the input port divided
  // by the size of the output ports provided in the constructor, in this case
  // 10 / 2 = 5.
  ASSERT_EQ(kNumOutputs, demux->num_output_ports());

  for (int output_index = 0; output_index < kNumOutputs; ++output_index) {
    const auto& output_vector =
        demux->get_output_port(output_index).Eval(*context);
    ASSERT_EQ(kOutputSize, output_vector.size());
    ASSERT_EQ(input_vector.segment<kOutputSize>(output_index * kOutputSize),
              output_vector);
  }
}

// Tests that the input signal gets demultiplexed into its individual
// components.
GTEST_TEST(VectorizedOutputSize, SizeDifferentFromOne) {
  const int kOutputPortOneSize = 1;
  const int kOutputPortTwoSize = 2;
  const int kOutputPortThreeSize = 3;
  const int kInputSize =
      kOutputPortOneSize + kOutputPortTwoSize + kOutputPortThreeSize;
  const std::vector<int> kOutputPortsSizes{
      kOutputPortOneSize, kOutputPortTwoSize, kOutputPortThreeSize};
  // Creates a demultiplexer with an input port of size six and number of output
  // ports of three. The size of each output port is specified inside the input
  // vector.
  auto demux = make_unique<Demultiplexer<double>>(
      kOutputPortsSizes /* output_ports_sizes */);
  auto context = demux->CreateDefaultContext();

  // Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context->num_input_ports());
  ASSERT_EQ(1, demux->num_input_ports());
  Eigen::VectorXd input_vector =
      Eigen::VectorXd::LinSpaced(kInputSize, 1.0, 6.0);

  // Hook input of the expected size.
  demux->get_input_port(0).FixValue(context.get(), input_vector);

  // The number of output ports must equal the length of the vector
  // kOutputPortsSizes.
  const int num_output_ports = kOutputPortsSizes.size();
  ASSERT_EQ(num_output_ports, demux->num_output_ports());

  int output_port_start = 0;
  for (int output_index = 0; output_index < num_output_ports; ++output_index) {
    const auto& output_vector =
        demux->get_output_port(output_index).Eval(*context);
    const int output_size = kOutputPortsSizes[output_index];
    ASSERT_EQ(output_size, output_vector.size());
    ASSERT_EQ(input_vector.segment(output_port_start, output_size),
              output_vector);
    output_port_start += output_size;
  }
}

// Tests that Demultiplexer allocates no state variables in the context_.
TEST_F(DemultiplexerTest, DemultiplexerIsStateless) {
  EXPECT_EQ(0, context_->num_continuous_states());
}

TEST_F(DemultiplexerTest, DirectFeedthrough) {
  EXPECT_TRUE(demux_->HasAnyDirectFeedthrough());
  for (int i = 0; i < demux_->num_output_ports(); ++i) {
    EXPECT_TRUE(demux_->HasDirectFeedthrough(0, i));
  }
}

// Tests converting to different scalar types.
TEST_F(DemultiplexerTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*demux_, [&](const auto& converted) {
    EXPECT_EQ(1, converted.num_input_ports());
    EXPECT_EQ(3, converted.num_output_ports());

    EXPECT_EQ(3, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_output_port(0).size());
    EXPECT_EQ(1, converted.get_output_port(1).size());
    EXPECT_EQ(1, converted.get_output_port(2).size());
  }));
}

TEST_F(DemultiplexerTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*demux_));
}

}  // namespace
}  // namespace systems
}  // namespace drake

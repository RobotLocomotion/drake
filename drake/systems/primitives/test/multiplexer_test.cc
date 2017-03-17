#include "drake/systems/primitives/multiplexer.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

class MultiplexerTest : public ::testing::Test {
 protected:
  void Reset(std::vector<int> input_sizes) {
    mux_ = make_unique<Multiplexer<double>>(input_sizes);
    context_ = mux_->CreateDefaultContext();
    output_ = mux_->AllocateOutput(*context_);
  }

  std::unique_ptr<System<double>> mux_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

TEST_F(MultiplexerTest, Basic) {
  Reset({2, 1, 3});

  // Confirm the shape.
  ASSERT_EQ(3, mux_->get_num_input_ports());
  ASSERT_EQ(2, mux_->get_input_port(0).size());
  ASSERT_EQ(1, mux_->get_input_port(1).size());
  ASSERT_EQ(3, mux_->get_input_port(2).size());
  ASSERT_EQ(3, context_->get_num_input_ports());
  ASSERT_EQ(1, mux_->get_num_output_ports());
  ASSERT_EQ(6, mux_->get_output_port(0).size());
  ASSERT_EQ(1, output_->get_num_ports());
  ASSERT_EQ(6, output_->get_vector_data(0)->size());

  // Provide input data.
  context_->FixInputPort(0, BasicVector<double>::Make({11.0, 22.0}));
  context_->FixInputPort(1, BasicVector<double>::Make({21.0}));
  context_->FixInputPort(2, BasicVector<double>::Make({31.0, 32.0, 33.0}));

  // Confirm output data.
  mux_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(6, output_->get_vector_data(0)->size());
  auto value = output_->get_vector_data(0)->get_value();
  ASSERT_EQ(11.0, value[0]);
  ASSERT_EQ(22.0, value[1]);
  ASSERT_EQ(21.0, value[2]);
  ASSERT_EQ(31.0, value[3]);
  ASSERT_EQ(32.0, value[4]);
  ASSERT_EQ(33.0, value[5]);
}

TEST_F(MultiplexerTest, ScalarConstructor) {
  mux_ = make_unique<Multiplexer<double>>(4);
  context_ = mux_->CreateDefaultContext();
  output_ = mux_->AllocateOutput(*context_);

  // Confirm the shape.
  ASSERT_EQ(4, mux_->get_num_input_ports());
  ASSERT_EQ(1, mux_->get_input_port(0).size());
  ASSERT_EQ(1, mux_->get_input_port(1).size());
  ASSERT_EQ(1, mux_->get_input_port(2).size());
  ASSERT_EQ(1, mux_->get_input_port(3).size());
  ASSERT_EQ(4, context_->get_num_input_ports());
  ASSERT_EQ(1, mux_->get_num_output_ports());
  ASSERT_EQ(4, mux_->get_output_port(0).size());
  ASSERT_EQ(1, output_->get_num_ports());
  ASSERT_EQ(4, output_->get_vector_data(0)->size());
}

TEST_F(MultiplexerTest, IsStateless) {
  Reset({1});
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

}  // namespace
}  // namespace systems
}  // namespace drake

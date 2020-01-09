#include "drake/systems/primitives/multiplexer.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

class MultiplexerTest : public ::testing::Test {
 protected:
  void InitializeFromSizes(std::vector<int> input_sizes) {
    mux_ = make_unique<Multiplexer<double>>(input_sizes);
    context_ = mux_->CreateDefaultContext();
  }

  void InitializeFromMyVector() {
    mux_ = make_unique<Multiplexer<double>>(MyVector2d());
    context_ = mux_->CreateDefaultContext();
  }

  std::unique_ptr<System<double>> mux_;
  std::unique_ptr<Context<double>> context_;
};

TEST_F(MultiplexerTest, Basic) {
  InitializeFromSizes({2, 1, 3});

  // Confirm the shape.
  ASSERT_EQ(3, mux_->num_input_ports());
  ASSERT_EQ(2, mux_->get_input_port(0).size());
  ASSERT_EQ(1, mux_->get_input_port(1).size());
  ASSERT_EQ(3, mux_->get_input_port(2).size());
  ASSERT_EQ(3, context_->num_input_ports());
  ASSERT_EQ(1, mux_->num_output_ports());
  ASSERT_EQ(6, mux_->get_output_port(0).size());

  // Provide input data.
  mux_->get_input_port(0).FixValue(context_.get(), Eigen::Vector2d(11.0, 22.0));
  mux_->get_input_port(1).FixValue(context_.get(), 21.0);
  mux_->get_input_port(2).FixValue(context_.get(),
                                   Eigen::Vector3d(31.0, 32.0, 33.0));

  // Confirm output data.
  const auto& value = mux_->get_output_port(0).Eval(*context_);
  ASSERT_EQ(6, value.size());
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

  // Confirm the shape.
  ASSERT_EQ(4, mux_->num_input_ports());
  ASSERT_EQ(1, mux_->get_input_port(0).size());
  ASSERT_EQ(1, mux_->get_input_port(1).size());
  ASSERT_EQ(1, mux_->get_input_port(2).size());
  ASSERT_EQ(1, mux_->get_input_port(3).size());
  ASSERT_EQ(4, context_->num_input_ports());
  ASSERT_EQ(1, mux_->num_output_ports());
  ASSERT_EQ(4, mux_->get_output_port(0).size());
}

TEST_F(MultiplexerTest, ModelVectorConstructor) {
  InitializeFromMyVector();

  // Confirm the shape.
  ASSERT_EQ(2, mux_->num_input_ports());
  ASSERT_EQ(1, mux_->get_input_port(0).size());
  ASSERT_EQ(1, mux_->get_input_port(1).size());
  ASSERT_EQ(2, context_->num_input_ports());
  ASSERT_EQ(1, mux_->num_output_ports());
  ASSERT_EQ(2, mux_->get_output_port(0).size());

  // Confirm that the vector is truly MyVector2d.
  mux_->get_input_port(0).FixValue(context_.get(), 0.0);
  mux_->get_input_port(1).FixValue(context_.get(), 0.0);
  ASSERT_NO_THROW(mux_->get_output_port(0).Eval<MyVector2d>(*context_));
}

TEST_F(MultiplexerTest, IsStateless) {
  InitializeFromSizes({1});
  EXPECT_EQ(0, context_->num_continuous_states());
}

TEST_F(MultiplexerTest, ToAutoDiffPass) {
  InitializeFromSizes({1, 1, 1});
  EXPECT_TRUE(is_autodiffxd_convertible(*mux_, [&](const auto& converted) {
    EXPECT_EQ(3, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());

    EXPECT_EQ(1, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_input_port(1).size());
    EXPECT_EQ(1, converted.get_input_port(2).size());
    EXPECT_EQ(3, converted.get_output_port(0).size());
  }));
}

TEST_F(MultiplexerTest, ToAutoDiffFail) {
  InitializeFromMyVector();
  // This is not yet supported, as getting the model_value subtype converted to
  // a different underlying scalar is not yet supported by the BasicVector API
  // (see #5454).
  EXPECT_FALSE(is_autodiffxd_convertible(*mux_));
}

TEST_F(MultiplexerTest, ToSymbolic) {
  InitializeFromSizes({1, 1, 1});
  EXPECT_TRUE(is_symbolic_convertible(*mux_, [&](const auto& converted) {
    EXPECT_EQ(3, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());

    EXPECT_EQ(1, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_input_port(1).size());
    EXPECT_EQ(1, converted.get_input_port(2).size());
    EXPECT_EQ(3, converted.get_output_port(0).size());
  }));
}

}  // namespace
}  // namespace systems
}  // namespace drake

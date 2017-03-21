#include "drake/systems/primitives/constant_vector_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using Eigen::Matrix;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class ConstantVectorSourceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_ = make_unique<ConstantVectorSource<double>>(kConstantVectorSource);
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* size */);
  }

  const Matrix<double, 2, 1, Eigen::DontAlign> kConstantVectorSource{2.0, 1.5};
  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

// Tests that the output of the ConstantVectorSource is correct.
TEST_F(ConstantVectorSourceTest, OutputTest) {
  // TODO(amcastro-tri): we should be able to ask:
  // source_->num_of_input_ports() after #3097.
  ASSERT_EQ(context_->get_num_input_ports(), 0);
  // TODO(amcastro-tri): we should be able to ask:
  // source_->num_of_output_ports() after #3097.
  ASSERT_EQ(output_->get_num_ports(), 1);

  source_->CalcOutput(*context_, output_.get());

  // TODO(amcastro-tri): Solve #3140 so that the next line reads:
  // auto& source_->get_output_vector(context, 0);
  // to directly get an Eigen expression.
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  EXPECT_TRUE(kConstantVectorSource.isApprox(
      output_vector->get_value(), Eigen::NumTraits<double>::epsilon()));
}

// Tests that ConstantVectorSource allocates no state variables in its context.
TEST_F(ConstantVectorSourceTest, ConstantVectorSourceIsStateless) {
  EXPECT_EQ(context_->get_continuous_state()->size(), 0);
}

}  // namespace
}  // namespace systems
}  // namespace drake

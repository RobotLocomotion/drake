#include "drake/systems/primitives/first_order_low_pass_filter.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

const double kTimeConstant = 3.0;
const int kSignalSize = 3;

class FirstOrderLowPassFilterTest : public ::testing::Test {
 protected:
  void SetUpSingleTimeConstantFilter() {
    filter_ = std::make_unique<FirstOrderLowPassFilter<double>>(
        kTimeConstant, kSignalSize);
    context_ = filter_->CreateDefaultContext();
    derivatives_ = filter_->AllocateTimeDerivatives();
    output_ = filter_->AllocateOutput(*context_);

    // Sets the state to zero initially.
    filter_->set_initial_output_value(
        context_.get(), Eigen::VectorXd::Zero(kSignalSize));
  }

  void SetUpMultipleTimeConstantsFilter() {
    Vector3<double> time_constants(4.0, 3.5, 3.0);
    filter_ = std::make_unique<FirstOrderLowPassFilter<double>>(time_constants);
    context_ = filter_->CreateDefaultContext();
    derivatives_ = filter_->AllocateTimeDerivatives();
    output_ = filter_->AllocateOutput(*context_);

    // Sets the state to zero initially.
    ContinuousState<double>* xc = continuous_state();
    EXPECT_EQ(kSignalSize, xc->size());
    EXPECT_EQ(kSignalSize, xc->get_misc_continuous_state().size());
    xc->SetFromVector(Eigen::VectorXd::Zero(kSignalSize));
  }

  ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  std::unique_ptr<FirstOrderLowPassFilter<double>> filter_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the system exports the correct topology.
TEST_F(FirstOrderLowPassFilterTest, Topology) {
  SetUpSingleTimeConstantFilter();
  ASSERT_EQ(1, filter_->get_num_input_ports());
  const auto& input_descriptor = filter_->get_input_port();
  EXPECT_EQ(kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(kSignalSize, input_descriptor.size());

  ASSERT_EQ(1, filter_->get_num_output_ports());
  const auto& output_descriptor = filter_->get_output_port();
  EXPECT_EQ(kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(kSignalSize, output_descriptor.size());
}

// Tests that the output of a low pass filter is its state.
TEST_F(FirstOrderLowPassFilterTest, Output) {
  SetUpSingleTimeConstantFilter();
  ASSERT_EQ(1, context_->get_num_input_ports());
  context_->FixInputPort(0, BasicVector<double>::Make({1.0, 2.0, 3.0}));

  filter_->CalcOutput(*context_, output_.get());

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_port);

  Eigen::Vector3d expected = Eigen::Vector3d::Zero();
  EXPECT_EQ(expected, output_port->get_value());

  continuous_state()->get_mutable_vector()->SetAtIndex(1, 42.0);
  expected << 0.0, 42.0, 0.0;
  filter_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(expected, output_port->get_value());
}

// Verifies the correctness of the time derivatives implementation.
TEST_F(FirstOrderLowPassFilterTest, Derivatives) {
  SetUpMultipleTimeConstantsFilter();
  ASSERT_EQ(1, context_->get_num_input_ports());
  Vector3<double> u({1.0, 2.0, 3.0});  // The input signal.
  context_->FixInputPort(0, u);

  // Sets a more interesting (non-zero) state.
  Vector3<double> z_expected(-1.0, 2.0, 3.5);
  filter_->set_initial_output_value(context_.get(), z_expected);

  filter_->CalcTimeDerivatives(*context_, derivatives_.get());
  filter_->CalcOutput(*context_, output_.get());

  // Current state.
  const BasicVector<double>* output_port = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_port);
  Vector3<double> z = continuous_state()->get_vector().CopyToVector();
  EXPECT_EQ(z_expected, z);

  auto time_constants = filter_->get_time_constants_vector();

  Eigen::Vector3d zdot_expected;  // Expected time derivatives.
  zdot_expected = (u - z).array() / time_constants.array();
  EXPECT_EQ(zdot_expected, derivatives_->CopyToVector());
}

// Asserts that low pass filters do not have any direct feedthrough inputs.
TEST_F(FirstOrderLowPassFilterTest, FilterIsNotDirectFeedthrough) {
  SetUpSingleTimeConstantFilter();
  EXPECT_FALSE(filter_->HasAnyDirectFeedthrough());
}

}  // namespace
}  // namespace systems
}  // namespace drake

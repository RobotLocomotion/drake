#include "drake/automotive/driving_command_mux.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace automotive {
namespace {

class DrivingCommandMuxTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mux_ = std::make_unique<DrivingCommandMux<double>>();
    context_ = mux_->CreateDefaultContext();
    output_ = mux_->AllocateOutput(*context_);
  }

  std::unique_ptr<DrivingCommandMux<double>> mux_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(DrivingCommandMuxTest, Basic) {
  // Confirm the shape.
  ASSERT_EQ(2, mux_->get_num_input_ports());
  ASSERT_EQ(1, mux_->steering_input().size());
  ASSERT_EQ(1, mux_->acceleration_input().size());
  ASSERT_EQ(1, mux_->get_num_output_ports());
  ASSERT_EQ(2, mux_->get_output_port(0).size());

  // Confirm the output is indeed a DrivingCommand.
  const DrivingCommand<double>* driving_command_output =
      dynamic_cast<const DrivingCommand<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, driving_command_output);

  // Provide input data.
  context_->FixInputPort(mux_->steering_input().get_index(),
                         systems::BasicVector<double>::Make({42.}));
  context_->FixInputPort(mux_->acceleration_input().get_index(),
                         systems::BasicVector<double>::Make({11.}));

  // Confirm output data.
  mux_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(42., driving_command_output->steering_angle());
  ASSERT_EQ(11., driving_command_output->acceleration());
}

TEST_F(DrivingCommandMuxTest, IsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state().size());
}

// Tests conversion to AutoDiffXd.
TEST_F(DrivingCommandMuxTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*mux_, [&](const auto& converted) {
    EXPECT_EQ(2, converted.get_num_input_ports());
    EXPECT_EQ(1, converted.get_num_output_ports());

    EXPECT_EQ(1, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_input_port(1).size());
    EXPECT_EQ(2, converted.get_output_port(0).size());

    const auto context = converted.CreateDefaultContext();
    const auto output = converted.AllocateOutput(*context);
    const DrivingCommand<AutoDiffXd>* driving_command_output =
        dynamic_cast<const DrivingCommand<AutoDiffXd>*>(
            output->get_vector_data(0));
    EXPECT_NE(nullptr, driving_command_output);
  }));
}

// Tests conversion to symbolic::Expression.
TEST_F(DrivingCommandMuxTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*mux_, [&](const auto& converted) {
    EXPECT_EQ(2, converted.get_num_input_ports());
    EXPECT_EQ(1, converted.get_num_output_ports());

    EXPECT_EQ(1, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_input_port(1).size());
    EXPECT_EQ(2, converted.get_output_port(0).size());

    const auto context = converted.CreateDefaultContext();
    const auto output = converted.AllocateOutput(*context);
    const DrivingCommand<symbolic::Expression>* driving_command_output =
        dynamic_cast<const DrivingCommand<symbolic::Expression>*>(
            output->get_vector_data(0));
    EXPECT_NE(nullptr, driving_command_output);
  }));
}

}  // namespace
}  // namespace automotive
}  // namespace drake

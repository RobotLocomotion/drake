#include "drake/automotive/idm_planner.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class IdmPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new IdmPlanner<double>(v_0_));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  double get_v_0_() const {
    return v_0_;
  }

  void SetInputValue(double x_ego, double v_ego,
                     double x_agent, double v_agent) {
    //auto input_ego = std::make_unique<IdmPlannerInput<double>>();
    //auto input_agent = std::make_unique<IdmPlannerInput<double>>();
    auto input_ego = std::make_unique<systems::BasicVector<double>>(2);
    auto input_agent = std::make_unique<systems::BasicVector<double>>(2);
    //input_ego->set_x(x_ego);
    //input_ego->set_v(v_ego);
    //input_agent->set_x(x_agent);
    //input_agent->set_v(v_agent);
    input_ego->SetAtIndex(0, x_ego); //set_x(x_ego);
    input_ego->SetAtIndex(1, v_ego); //set_v(v_ego);
    input_agent->SetAtIndex(0, x_agent); //set_x(x_agent);
    input_agent->SetAtIndex(1, v_agent); //set_v(v_agent);
    context_->SetInputPort(
        0, std::make_unique<systems::FreestandingInputPort>(
                                                       std::move(input_ego)));
    context_->SetInputPort(
        1, std::make_unique<systems::FreestandingInputPort>(
                                                       std::move(input_agent)));
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;

 private:
  const double v_0_ = 10.0;
};

TEST_F(IdmPlannerTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_ports().at(0);
    EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(systems::kInputPort, input_descriptor.get_face());
  //EXPECT_EQ(IdmPlannerInputIndices::kNumCoordinates,
  //          input_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, input_descriptor.get_sampling());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(1, output_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(IdmPlannerTest, Input) {
  // Define a pointer to where the EvalOutput results end up.
  const auto result = output_->get_vector_data(0);
  ASSERT_NE(nullptr, result);

  // Model parameters.
  const double a = 1.0;
  const double b = 3.0;
  const double s_0 = 1.0;
  const double time_headway = 0.1;
  const double delta = 4.0;
  const double l_a = 4.5;

  // Trial input values.
  const double x_e = 6.3;
  const double v_e = 4.2;
  const double x_a = 9.7;
  const double v_a = 13.1;

  // Set the inputs to some arbitrary values.
  SetInputValue(x_e, v_e, x_a, v_a);

  const double expected_output =
    a * (1.0 - pow(v_e / IdmPlannerTest::get_v_0_(), delta) -
           pow((s_0 + v_e * time_headway +
                v_e * (v_e - v_a) /
                (2 * sqrt(a * b))) /
               (x_a - x_e - l_a), 2.0));

  // Verify that the starting input is zero.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(expected_output, result->GetAtIndex(0));
}

}  // namespace
}  // namespace automotive
}  // namespace drake

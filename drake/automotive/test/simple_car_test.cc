#include "drake/automotive/simple_car.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class SimpleCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new SimpleCar<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    SetInputValue(0, 0, 0);
  }

  void SetInputValue(double steering_angle, double throttle, double brake) {
    auto value = std::make_unique<DrivingCommand<double>>();
    value->set_steering_angle(steering_angle);
    value->set_throttle(throttle);
    value->set_brake(brake);
    context_->SetInputPort(
        0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));
  }

  SimpleCarState<double>* continuous_state() {
    auto result = dynamic_cast<SimpleCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(SimpleCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(DrivingCommandIndices::kNumCoordinates, input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, output_descriptor.size());
}

TEST_F(SimpleCarTest, Output) {
  // Grab a pointer to where the CalcOutput results end up.
  const SimpleCarState<double>* const result =
      dynamic_cast<const SimpleCarState<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // New state just propagates through.
  continuous_state()->set_x(1.0);
  continuous_state()->set_y(2.0);
  continuous_state()->set_heading(3.0);
  continuous_state()->set_velocity(4.0);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x());
  EXPECT_EQ(2.0, result->y());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->velocity());

  // The input doesn't matter.
  SetInputValue(0.3, 0.5, 0.7);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x());
  EXPECT_EQ(2.0, result->y());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->velocity());

  // If the integrator does a naughty, we don't let it bleed through.
  // Heading should remain unchanged; velocity output clamps to zero.
  continuous_state()->set_velocity(-0.001);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x());
  EXPECT_EQ(2.0, result->y());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());
}

TEST_F(SimpleCarTest, Derivatives) {
  const double kTolerance = 1e-10;

  SimpleCarConfig<double> default_config;
  SimpleCar<double>::SetDefaultParameters(&default_config);
  const double wheelbase = default_config.wheelbase();
  const double max_abs_steering_angle = default_config.max_abs_steering_angle();
  const double max_abs_curvature = std::tan(max_abs_steering_angle) / wheelbase;
  const double max_acceleration = default_config.max_acceleration();

  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const SimpleCarState<double>* const result =
      dynamic_cast<const SimpleCarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting state is all zeros.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // Half throttle yields half of the max acceleration.
  SetInputValue(0.0, 0.5, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(0.5 * max_acceleration, result->velocity(), kTolerance);

  // Set speed to mid-range, with zero input.
  continuous_state()->set_velocity(10.0);
  SetInputValue(0.0, 0.0, 0.0);
  // At heading 0, we are moving along +x.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // A non-zero steering_angle turns in the same direction.  We'd like to turn
  // at 0.1 rad/s at a speed of 10m/s, so we want a curvature of 0.01.
  const double steering_angle = std::atan(0.01 * wheelbase);
  SetInputValue(steering_angle, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(0.1, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());
  SetInputValue(-steering_angle, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(-0.1, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());

  // Very large steering angles are clamped at the limit.
  SetInputValue(M_PI_2, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(10.0 * max_abs_curvature, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());
  SetInputValue(-M_PI_2, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(-10.0 * max_abs_curvature, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());

  // Half brake yields half of the max acceleration.
  SetInputValue(0.0, 0.0, 0.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(-0.5 * max_acceleration, result->velocity(), kTolerance);

  // Throttle and brake offset each other.
  SetInputValue(0.0, 0.2, 0.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(10.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(-0.3 * max_acceleration, result->velocity(), kTolerance);

  // When velocity is past max_speed, a damping term takes over.
  const double too_fast = default_config.max_velocity() + 0.001;
  continuous_state()->set_velocity(too_fast);
  SetInputValue(0.0, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(too_fast, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(-0.001 * default_config.velocity_limit_kp(),
              result->velocity(), kTolerance);
  // ... but not when the brake is larger.
  SetInputValue(0.0, 0.0, 0.1);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(too_fast, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(-0.1 * max_acceleration, result->velocity(), kTolerance);

  // When velocity is below zero, a damping term takes over.
  const double backwards = -0.001;
  continuous_state()->set_velocity(backwards);
  SetInputValue(M_PI_2, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());   // N.B. Not -0.001!
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());  // N.B. Not rotating!
  EXPECT_NEAR(0.001 * default_config.velocity_limit_kp(),
              result->velocity(), kTolerance);
  // ... but not when the throttle is larger.
  SetInputValue(M_PI_2, 0.1, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());  // N.B. Still zero.
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());  // N.B. Still zero.
  EXPECT_NEAR(0.1 * max_acceleration, result->velocity(), kTolerance);

  // When velocity is near zero, the deceleration is diminished via tanh.  (We
  // can remove this test once the tanh in the implementation goes away, to be
  // replaced with zero-crossing events.)
  continuous_state()->set_velocity(0.1);
  // We apply 100% brake, but the deceleration is no stronger than 95%.
  SetInputValue(0.0, 0.0, 1.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.1, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_LT(-0.95, result->velocity() / max_acceleration);
  // At even slower speeds, the deceleration is even weaker.
  {
    const double prior_acceleration = result->velocity();
    continuous_state()->set_velocity(0.01);
    dut_->CalcTimeDerivatives(*context_, derivatives_.get());
    EXPECT_LT(-0.5, result->velocity() / max_acceleration);
    // As velocity approaches zero, the deceleration is monotonically weaker.
    EXPECT_LT(prior_acceleration, result->velocity());
  }
  // The diminishment does not when happen accelerating.
  continuous_state()->set_velocity(0.1);
  SetInputValue(0.0, 1.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.1, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(max_acceleration, result->velocity(), kTolerance);

  // A heading of +90deg points us at +y.
  continuous_state()->set_velocity(10.0);
  continuous_state()->set_heading(0.5 * M_PI);
  SetInputValue(0.0, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(0.0, result->x(), kTolerance);
  EXPECT_NEAR(10.0, result->y(), kTolerance);
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());
}

}  // namespace
}  // namespace automotive
}  // namespace drake

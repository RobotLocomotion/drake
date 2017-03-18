#include "drake/automotive/simple_car.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/symbolic_formula.h"

namespace drake {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

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
    context_->FixInputPort(0, std::move(value));
  }

  SimpleCarState<double>* continuous_state() {
    auto result = dynamic_cast<SimpleCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  const SimpleCarState<double>* state_output() const {
    auto state = dynamic_cast<const SimpleCarState<double>*>(
        output_->get_vector_data(0));
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const PoseVector<double>* pose_output() const {
    auto pose = dynamic_cast<const PoseVector<double>*>(
        output_->get_vector_data(1));
    DRAKE_DEMAND(pose != nullptr);
    return pose;
  }

  const FrameVelocity<double>* velocity_output() const {
    auto velocity = dynamic_cast<const FrameVelocity<double>*>(
        output_->get_vector_data(2));
    DRAKE_DEMAND(velocity != nullptr);
    return velocity;
  }


  // Sets an arbitrary, nonzero state.
  void InitializeNonzeroState() {
    continuous_state()->set_x(1.0);
    continuous_state()->set_y(2.0);
    continuous_state()->set_heading(3.0);
    continuous_state()->set_velocity(4.0);
  }

  // Checks that the pose output has the correct values for the state that
  // InitializeNonzeroState sets.
  void VerifyNonzeroPose() {
    Eigen::Translation<double, 3> p_WC = pose_output()->get_translation();
    EXPECT_EQ(1.0, p_WC.translation().x());
    EXPECT_EQ(2.0, p_WC.translation().y());
    EXPECT_EQ(0.0, p_WC.translation().z());

    // A rotation about the z axis is nonzero in only the real and k parts of
    // q = w + xi + yj + zk.
    Eigen::Quaternion<double> R_WC = pose_output()->get_rotation();
    EXPECT_EQ(std::cos(3.0 / 2), R_WC.w());
    EXPECT_EQ(0.0, R_WC.x());
    EXPECT_EQ(0.0, R_WC.y());
    EXPECT_EQ(std::sin(3.0 / 2), R_WC.z());
  }

  // Checks that the velocity output has the correct values for the state that
  // InitializeNonzeroState sets.
  void VerifyNonzeroVelocity() {
    const multibody::SpatialVelocity<double> v_WC =
        velocity_output()->get_velocity();
    EXPECT_EQ(std::cos(3.0) * 4.0, v_WC.translational()[0]);
    EXPECT_EQ(std::sin(3.0) * 4.0, v_WC.translational()[1]);
  }

  std::unique_ptr<SimpleCar<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(SimpleCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(DrivingCommandIndices::kNumCoordinates, input_descriptor.size());

  ASSERT_EQ(3, dut_->get_num_output_ports());
  const auto& state_output = dut_->state_output();
  EXPECT_EQ(systems::kVectorValued, state_output.get_data_type());
  EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, state_output.size());

  const auto& pose_output = dut_->pose_output();
  EXPECT_EQ(systems::kVectorValued, pose_output.get_data_type());
  EXPECT_EQ(PoseVector<double>::kSize, pose_output.size());

  const auto& velocity_output = dut_->velocity_output();
  EXPECT_EQ(systems::kVectorValued, velocity_output.get_data_type());
  EXPECT_EQ(FrameVelocity<double>::kSize, velocity_output.size());

  // This test covers a portion of the symbolic::Expression instantiation.
  ASSERT_FALSE(dut_->HasAnyDirectFeedthrough());
}

TEST_F(SimpleCarTest, ZeroOutput) {
  auto state = state_output();
  auto pose = pose_output();

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, state->x());
  EXPECT_EQ(0.0, state->y());
  EXPECT_EQ(0.0, state->heading());
  EXPECT_EQ(0.0, state->velocity());

  EXPECT_TRUE(CompareMatrices(Isometry3<double>::Identity().matrix(),
                              pose->get_isometry().matrix()));
}

TEST_F(SimpleCarTest, StateAppearsInOutput) {
  // New state just propagates through.
  InitializeNonzeroState();

  auto state = state_output();

  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, state->x());
  EXPECT_EQ(2.0, state->y());
  EXPECT_EQ(3.0, state->heading());
  EXPECT_EQ(4.0, state->velocity());
  VerifyNonzeroPose();
  VerifyNonzeroVelocity();
}

TEST_F(SimpleCarTest, InputDoesNotAffectOutput) {
  InitializeNonzeroState();

  // The input doesn't matter.
  SetInputValue(0.3, 0.5, 0.7);

  auto state = state_output();

  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, state->x());
  EXPECT_EQ(2.0, state->y());
  EXPECT_EQ(3.0, state->heading());
  EXPECT_EQ(4.0, state->velocity());
  VerifyNonzeroPose();
  VerifyNonzeroVelocity();
}

TEST_F(SimpleCarTest, OutputVelocityIsClamped) {
  InitializeNonzeroState();

  // If the integrator does a naughty, we don't let it bleed through.
  // Heading should remain unchanged; velocity output clamps to zero.
  continuous_state()->set_velocity(-0.001);

  auto state = state_output();

  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, state->x());
  EXPECT_EQ(2.0, state->y());
  EXPECT_EQ(3.0, state->heading());
  EXPECT_EQ(0.0, state->velocity());
  VerifyNonzeroPose();
  for (int i = 0; i < FrameVelocity<double>::kSize; ++i) {
    EXPECT_EQ(0.0, velocity_output()->GetAtIndex(i));
  }
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

TEST_F(SimpleCarTest, TransmogrifyAutoDiff) {
  const auto& other_dut = dut_->ToAutoDiffXd();
  ASSERT_NE(other_dut.get(), nullptr);

  auto other_context = other_dut->CreateDefaultContext();
  auto other_output = other_dut->AllocateOutput(*other_context);
  auto other_derivatives = other_dut->AllocateTimeDerivatives();

  auto input_value = std::make_unique<DrivingCommand<AutoDiffXd>>();
  other_context->FixInputPort(0, std::move(input_value));

  // For now, running without exceptions is good enough.
  other_dut->CalcOutput(*other_context, other_output.get());
  other_dut->CalcTimeDerivatives(*other_context, other_derivatives.get());
}

TEST_F(SimpleCarTest, TransmogrifySymbolic) {
  const auto& other_dut = dut_->ToSymbolic();
  ASSERT_NE(other_dut.get(), nullptr);

  auto other_context = other_dut->CreateDefaultContext();
  auto other_output = other_dut->AllocateOutput(*other_context);
  auto other_derivatives = other_dut->AllocateTimeDerivatives();

  // TODO(jwnimmer-tri) We should have a framework way to just say "make the
  // entire context symbolic variables (vs zero)" that is reusable for any
  // consumer of the framework.
  auto input_value = std::make_unique<DrivingCommand<symbolic::Expression>>();
  other_context->FixInputPort(0, std::move(input_value));
  systems::VectorBase<symbolic::Expression>& xc =
      *other_context->get_mutable_continuous_state_vector();
  for (int i = 0; i < xc.size(); ++i) {
    xc[i] = symbolic::Variable("xc" + std::to_string(i));
  }

  // For now, running without exceptions is good enough.
  other_dut->CalcOutput(*other_context, other_output.get());
  other_dut->CalcTimeDerivatives(*other_context, other_derivatives.get());
}

}  // namespace
}  // namespace automotive
}  // namespace drake

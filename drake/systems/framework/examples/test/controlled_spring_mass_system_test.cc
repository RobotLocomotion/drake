#include "drake/systems/framework/examples/controlled_spring_mass_system.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

const double kSpring = 300.0;  // N/m
const double kMass = 2.0;      // kg
const double Kp = 1.0;  // Controller's proportional constant.
const double Kd = 1.0;  // Controller's derivative constant.
const double Ki = 1.0;  // Controller's integral constant.
const double x_target = 1.0;  // The target position.

class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ =
        make_unique<PidControlledSpringMassSystem<double>>(
            kSpring, kMass, Kp, Ki, Kd, x_target);

    context_ = model_->CreateDefaultContext();
    output_ = model_->AllocateOutput(*context_);

    // Initialize to default conditions.
    model_->SetDefaultState(context_.get());
  }

  // Returns the continuous state of the given @p system.
  ContinuousState<double>* GetMutableContinuousState(
      const System<double>* system) {
    return model_->GetMutableSubsystemState(context_.get(), system)
        ->continuous_state.get();
  }

  std::unique_ptr<PidControlledSpringMassSystem<double>> model_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, EvalOutput) {
  // Sets a non-zero initial condition.
  model_->set_position(context_.get(), 2.0);
  model_->set_velocity(context_.get(), -1.0);

  model_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(1, output_->get_num_ports());
  Eigen::Vector3d expected_output(2.0, -1.0, 0.0);

  const BasicVector<double>* output =
      dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output);
  EXPECT_EQ(expected_output[0], output->get_value()[0]);
  EXPECT_EQ(expected_output[1], output->get_value()[1]);
  EXPECT_EQ(expected_output[2], output->get_value()[2]);
}

TEST_F(DiagramTest, EvalTimeDerivatives) {
  std::unique_ptr<ContinuousState<double>> derivatives =
      model_->AllocateTimeDerivatives();

  const double x0 = 2.0;
  const double v0 = -1.5;

  // Sets a non-zero initial condition.
  model_->set_position(context_.get(), x0);
  model_->set_velocity(context_.get(), v0);

  model_->EvalTimeDerivatives(*context_, derivatives.get());

  // The spring-mass plant has a state vector of size 3. One position, one
  // velocity and one misc state (energy). In addition, the model has an
  // addition misc state corresponding to the integral of the PID controller.
  // Therefore the size of the misc state vector is 2.
  ASSERT_EQ(4, derivatives->get_state().size());
  ASSERT_EQ(1, derivatives->get_generalized_position().size());
  ASSERT_EQ(1, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(2, derivatives->get_misc_continuous_state().size());

  // The derivatives of plant.
  const ContinuousState<double>& plant_xcdot =
      model_->GetSubsystemDerivatives(*derivatives, &model_->get_plant());
  // Position derivative.
  EXPECT_EQ(v0, plant_xcdot.get_state().GetAtIndex(0));

  // Acceleration.
  const double error = x0 - x_target;
  const double error_rate = v0;  // target velocity is zero.
  const double pid_actuation = Kp * error +  Kd * error_rate;
  EXPECT_EQ((-kSpring * x0 - pid_actuation) / kMass,
            plant_xcdot.get_state().GetAtIndex(1));

  // Work.
  EXPECT_EQ(model_->get_plant().EvalConservativePower(*context_),
            plant_xcdot.get_state().GetAtIndex(2));
}

}  // namespace
}  // namespace systems
}  // namespace drake

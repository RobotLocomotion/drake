#include "drake/systems/analysis/test/controlled_spring_mass_system/controlled_spring_mass_system.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

using std::make_unique;

namespace drake {
namespace systems {
namespace {

const double kSpring = 300.0;  // N/m
const double kMass = 2.0;      // kg
const double kProportionalConstant = 1.0;  // N/m
const double kDerivativeConstant = 1.0;  // N*s/m
const double kIntegralConstant = 1.0;  // N/(m*s)
const double kTargetPosition = 1.0;  // m

// A unit test fixture to evaluate the correct functioning of the
// PidControlledSpringMassSystem example.
class SpringMassSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ =
        make_unique<PidControlledSpringMassSystem<double>>(
            kSpring, kMass,
            kProportionalConstant, kIntegralConstant, kDerivativeConstant,
            kTargetPosition);

    model_context_ = model_->CreateDefaultContext();
    output_ = model_->AllocateOutput(*model_context_);

    // Gets the plant subcontext.
    plant_context_ =
        model_->GetMutableSubsystemContext(
            model_context_.get(), &model_->get_plant());
  }

  std::unique_ptr<PidControlledSpringMassSystem<double>> model_;
  std::unique_ptr<Context<double>> model_context_;
  Context<double>* plant_context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests the correct output from the model.
TEST_F(SpringMassSystemTest, EvalOutput) {
  const double x0 = 2.0;
  const double v0 = -1.0;
  // Sets a non-zero initial condition.
  model_->set_position(model_context_.get(), x0);
  model_->set_velocity(model_context_.get(), v0);

  ASSERT_EQ(1, output_->get_num_ports());
  model_->CalcOutput(*model_context_, output_.get());

  // Output equals the state of the spring-mass plant being controlled which
  // consists of position, velocity and energy.
  Eigen::Vector3d expected_output(x0, v0, 0.0);

  const BasicVector<double>* output = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output);
  EXPECT_EQ(expected_output[0], output->get_value()[0]);
  EXPECT_EQ(expected_output[1], output->get_value()[1]);
  EXPECT_EQ(expected_output[2], output->get_value()[2]);
}

TEST_F(SpringMassSystemTest, EvalTimeDerivatives) {
  const double x0 = 2.0;
  const double v0 = -1.5;

  // Sets a non-zero initial condition.
  model_->set_position(model_context_.get(), x0);
  model_->set_velocity(model_context_.get(), v0);

  std::unique_ptr<ContinuousState<double>> derivatives =
      model_->AllocateTimeDerivatives();
  model_->CalcTimeDerivatives(*model_context_, derivatives.get());

  // The spring-mass plant has a state vector of size 3. One position, one
  // velocity and one miscellaneous state (energy). Moreover, the model has an
  // additional miscellaneous state corresponding to the integral of the PID
  // controller.Therefore the size of the misc state vector is 2.
  ASSERT_EQ(4, derivatives->size());
  ASSERT_EQ(1, derivatives->get_generalized_position().size());
  ASSERT_EQ(1, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(2, derivatives->get_misc_continuous_state().size());

  // The derivatives of plant.
  const ContinuousState<double>* plant_xcdot =
      model_->GetSubsystemDerivatives(*derivatives, &model_->get_plant());

  // Position derivative.
  EXPECT_EQ(v0, plant_xcdot->get_vector().GetAtIndex(0));

  // Acceleration.
  const double error = x0 - kTargetPosition;
  const double error_rate = v0;  // target velocity is zero.
  const double pid_actuation =
      kProportionalConstant * error +  kDerivativeConstant * error_rate;
  EXPECT_EQ((-kSpring * x0 - pid_actuation) / kMass,
            plant_xcdot->get_vector().GetAtIndex(1));

  // Power.
  EXPECT_EQ(model_->get_plant().EvalConservativePower(*plant_context_),
            plant_xcdot->get_vector().GetAtIndex(2));
}

}  // namespace
}  // namespace systems
}  // namespace drake

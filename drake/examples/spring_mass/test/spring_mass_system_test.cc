#include "drake/examples/spring_mass/spring_mass_system.h"

#include <memory>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/state_subvector.h"
#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"
#include "drake/util/eigen_matrix_compare.h"

namespace drake {

using systems::Context;
using systems::ContinuousState;
using systems::LeafStateVector;
using systems::StateSubvector;
using systems::StateVector;
using systems::SystemOutput;
using systems::VectorInterface;
using util::MatrixCompareType;

namespace examples {
namespace {

const double kSpring = 300.0;  // N/m
const double kMass = 2.0;      // kg

class SpringMassSystemTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Construct the system I/O objects.
    system_.reset(new SpringMassSystem("test_system", kSpring, kMass));
    context_ = system_->CreateDefaultContext();
    system_output_ = system_->AllocateOutput();
    erased_derivatives_ = system_->AllocateStateDerivatives();

    // Set up some convenience pointers.
    state_ = dynamic_cast<SpringMassStateVector*>(
        context_->get_mutable_state()->continuous_state->get_mutable_state());
    output_ = dynamic_cast<const SpringMassOutputVector*>(
        system_output_->ports[0]->get_vector_data());
    derivatives_ =
        dynamic_cast<SpringMassStateVector*>(erased_derivatives_.get());
  }

  void InitializeState(const double position, const double velocity) {
    state_->set_position(position);
    state_->set_velocity(velocity);
  }

 protected:
  std::unique_ptr<SpringMassSystem> system_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> system_output_;
  SpringMassStateVector* state_;
  const SpringMassOutputVector* output_;
  SpringMassStateVector* derivatives_;

 private:
  std::unique_ptr<StateVector<double>> erased_derivatives_;
};

TEST_F(SpringMassSystemTest, Name) {
  EXPECT_EQ("test_system", system_->get_name());
}

TEST_F(SpringMassSystemTest, CloneState) {
  InitializeState(1.0, 2.0);
  std::unique_ptr<LeafStateVector<double>> clone = state_->Clone();
  SpringMassStateVector* typed_clone =
      dynamic_cast<SpringMassStateVector*>(clone.get());
  ASSERT_NE(nullptr, typed_clone);
  EXPECT_EQ(1.0, typed_clone->get_position());
  EXPECT_EQ(2.0, typed_clone->get_velocity());
}

TEST_F(SpringMassSystemTest, CloneOutput) {
  InitializeState(1.0, 2.0);
  system_->Output(*context_, system_output_.get());
  std::unique_ptr<VectorInterface<double>> clone = output_->Clone();

  SpringMassOutputVector* typed_clone =
      dynamic_cast<SpringMassOutputVector*>(clone.get());
  EXPECT_EQ(1.0, typed_clone->get_position());
  EXPECT_EQ(2.0, typed_clone->get_velocity());
}

// Tests that state is passed through to the output.
TEST_F(SpringMassSystemTest, Output) {
  InitializeState(0.1, 0.0);  // Displacement 100cm, no velocity.
  system_->Output(*context_, system_output_.get());
  ASSERT_EQ(1, system_output_->ports.size());

  // Check the output through the application-specific interface.
  EXPECT_NEAR(0.1, output_->get_position(), 1e-8);
  EXPECT_EQ(0.0, output_->get_velocity());

  // Check the output through the VectorInterface API.
  ASSERT_EQ(2, output_->size());
  EXPECT_NEAR(0.1, output_->get_value()[0], 1e-8);
  EXPECT_EQ(0.0, output_->get_value()[1]);
}

// Tests that second-order structure is exposed in the state.
TEST_F(SpringMassSystemTest, SecondOrderStructure) {
  InitializeState(1.2, 3.4);  // Displacement 1.2m, velocity 3.4m/sec.
  ContinuousState<double>* continuous_state =
      context_->get_mutable_state()->continuous_state.get();
  ASSERT_EQ(1, continuous_state->get_generalized_position().size());
  ASSERT_EQ(1, continuous_state->get_generalized_velocity().size());
  ASSERT_EQ(0, continuous_state->get_misc_continuous_state().size());
  EXPECT_NEAR(1.2, continuous_state->get_generalized_position().GetAtIndex(0),
              1e-8);
  EXPECT_NEAR(3.4, continuous_state->get_generalized_velocity().GetAtIndex(0),
              1e-8);
}

// Tests that second-order structure can be processed in
// MapVelocityToConfigurationDerivative.
TEST_F(SpringMassSystemTest, MapVelocityToConfigurationDerivative) {
  InitializeState(1.2, 3.4);  // Displacement 1.2m, velocity 3.4m/sec.
  ContinuousState<double>* continuous_state =
      context_->get_mutable_state()->continuous_state.get();

  // Slice just the configuration derivatives out of the derivative
  // vector.
  StateSubvector<double> configuration_derivatives(derivatives_, 0, 1);

  system_->MapVelocityToConfigurationDerivatives(
      *context_, continuous_state->get_generalized_velocity(),
      &configuration_derivatives);

  EXPECT_NEAR(3.4, derivatives_->get_position(), 1e-8);
  EXPECT_NEAR(3.4, configuration_derivatives.GetAtIndex(0), 1e-8);
}

TEST_F(SpringMassSystemTest, ForcesPositiveDisplacement) {
  InitializeState(0.1, 0.1);  // Displacement 0.1m, velocity 0.1m/sec.
  system_->Dynamics(*context_, derivatives_);

  ASSERT_EQ(2, derivatives_->size());
  // The derivative of position is velocity.
  EXPECT_NEAR(0.1, derivatives_->get_position(), 1e-8);
  // The derivative of velocity is force over mass.
  EXPECT_NEAR(-kSpring * 0.1 / kMass, derivatives_->get_velocity(), 1e-8);
}

TEST_F(SpringMassSystemTest, ForcesNegativeDisplacement) {
  InitializeState(-0.1, 0.2);  // Displacement -0.1m, velocity 0.2m/sec.
  system_->Dynamics(*context_, derivatives_);

  ASSERT_EQ(2, derivatives_->size());
  // The derivative of position is velocity.
  EXPECT_NEAR(0.2, derivatives_->get_position(), 1e-8);
  // The derivative of velocity is force over mass.
  EXPECT_NEAR(-kSpring * -0.1 / kMass, derivatives_->get_velocity(), 1e-8);
}

// TODO(david-german-tri, sherm1): Add test cases using integration.

}  // namespace
}  // namespace examples
}  // namespace drake

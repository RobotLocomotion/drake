#include "drake/examples/particle1d/particle1d_plant.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace particle1d {
namespace {

// Test SetConstantParameters method which sets the mass of the particle
// from the .urdf file.
GTEST_TEST(ParticlePlantTest, SetConstantParametersTest) {
  // Parsing the URDF and constructing a RigidBodyTree from it
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/particle1d/particle1d.urdf"),
      multibody::joints::kFixed, tree.get());

  Particle1dPlant<double> test_particle;

  test_particle.SetConstantParameters(*tree);

  // The particles mass is set in the .urdf file as 3 kg.
  EXPECT_EQ(test_particle.get_mass(), 3);
  EXPECT_NE(test_particle.get_mass(), 1);
}

// Verifying that the output of the plant is a vector consistent with the
// state. The plant has only one output port and has the following values:
// y0 == x and y1 == ẋ.
GTEST_TEST(ParticlePlantTest, OutputTest) {
  Particle1dPlant<double> test_particle;

  // Create a context and allocate an output port for the plant.
  auto context = test_particle.CreateDefaultContext();
  auto output = test_particle.AllocateOutput(*context);

  // Get the continuous state vector from the context and arbitrarily set the
  // state to [0.5, 0.7].
  systems::VectorBase<double>& state =
      context->get_mutable_continuous_state_vector();
  state.SetAtIndex(0, 0.5);
  state.SetAtIndex(1, 0.7);

  // Compute outputs, which should simply be the state.
  test_particle.CalcOutput(*context, output.get());

  // Get the output vector at port 0.
  systems::BasicVector<double>* output_vector = output->GetMutableVectorData(0);

  // Check output is equal to the input.
  EXPECT_EQ(output_vector->GetAtIndex(0), state.GetAtIndex(0));
  EXPECT_EQ(output_vector->GetAtIndex(1), state.GetAtIndex(1));
}

// Verify that the derivative calculation of the Particle1dPlant is
// the time derivative of the state (expect ẋ and ẍ).
GTEST_TEST(ParticlePlantTest, DerivativesTest) {
  Particle1dPlant<double> test_particle;

  // Create a context and allocate the time derivatives.
  auto context = test_particle.CreateDefaultContext();
  context->SetTimeStateAndParametersFrom(*context);
  auto derivatives = test_particle.AllocateTimeDerivatives();

  // Get the continuous state vector from the system context.
  systems::VectorBase<double>& state =
      context->get_mutable_continuous_state_vector();

  // Please note, the state vector has the following assignments:
  // state[0] = x and state[1] = ẋ.
  int x_index = 0;
  int xDt_index = 1;

  // Set the initial state arbitrarily to zero.
  state.SetAtIndex(x_index, 0);
  state.SetAtIndex(xDt_index, 0);

  // Set initial time to zero.
  context->set_time(0.0);
  double time = context->get_time();

  // Compute derivatives.
  test_particle.CalcTimeDerivatives(*context, derivatives.get());
  const systems::VectorBase<double>& derivatives_vector =
      derivatives->get_vector();

  const double mass = test_particle.get_mass();

  // Check derivative results against the analytical solution.
  EXPECT_EQ(derivatives_vector.GetAtIndex(0),
            sin(time) / mass);  // ẋ = sin(t)/m.
  EXPECT_EQ(derivatives_vector.GetAtIndex(1),
            cos(time) / mass);  // ẍ = cos(t)/m.

  // Test the derivatives at time = 3 seconds.
  context->set_time(3.0);
  time = context->get_time();

  // Set the state assuming the system has evolved over time.
  state.SetAtIndex(x_index, (-cos(time) + 1) / mass);  // x = (-cos(t) + 1)/m.
  state.SetAtIndex(xDt_index, sin(time) / mass);       // ẋ = sin(t)/m

  // Compute derivatives.
  test_particle.CalcTimeDerivatives(*context, derivatives.get());
  const systems::VectorBase<double>& new_derivatives_vector =
      derivatives->get_vector();

  // Check derivative results against the analytical solution.
  EXPECT_EQ(new_derivatives_vector.GetAtIndex(0),
            sin(time) / mass);  // ẋ = sin(t)/m.
  EXPECT_EQ(new_derivatives_vector.GetAtIndex(1),
            cos(time) / mass);  // ẍ = cos(t)/m.
}

// Verify that the system can be converted to an AutoDiffXd system.
GTEST_TEST(ParticlePlantTest, ToAutoDiffTest) {
  Particle1dPlant<double> test_particle;

  auto context = test_particle.CreateDefaultContext();
  auto& state = context->get_mutable_continuous_state_vector();

  // Set the context time to an arbitrary time of 3 seconds.
  context->set_time(3.0);
  auto time = context->get_time();

  auto mass = test_particle.get_mass();

  // The state vector has the following assignments:
  // state[0] = x and state[1] = ẋ.
  int x_index = 0;
  int xDt_index = 1;

  // Set the state of the system.
  state.SetAtIndex(x_index, (-cos(time) + 1) / mass);  // x = (-cos(t) + 1)/m.
  state.SetAtIndex(xDt_index, sin(time) / mass);       // ẋ = sin(t)/m

  // Transmogrify the particle to autodiff.
  std::unique_ptr<Particle1dPlant<AutoDiffXd>> ad_test_particle =
      systems::System<double>::ToAutoDiffXd(test_particle);

  // Construct a new context based on AutoDiff.
  auto ad_context = ad_test_particle->CreateDefaultContext();
  ad_context->SetTimeStateAndParametersFrom(*context);
  auto ad_derivatives = ad_test_particle->AllocateTimeDerivatives();

  auto& ad_state = ad_context->get_mutable_continuous_state_vector();

  // Check that the transformed AutoDiff vector has the correct values and that
  // the derivatives vector has been created.
  EXPECT_EQ(ad_state[0].value(), state[0]);
  EXPECT_EQ(ad_state[1].value(), state[1]);
  EXPECT_EQ(ad_state[0].derivatives().size(), 0);
  EXPECT_EQ(ad_state[1].derivatives().size(), 0);

  // Compute derivatives.
  ad_test_particle->CalcTimeDerivatives(*ad_context, ad_derivatives.get());

  const systems::VectorBase<AutoDiffXd >& new_derivatives_vector =
      ad_derivatives->get_vector();

  // Check derivative results against the analytical solution.
  EXPECT_EQ(new_derivatives_vector[0].value(),
            sin(time) / mass);  // ẋ = sin(t)/m.
  EXPECT_EQ(new_derivatives_vector[1].value(),
            cos(time) / mass);  // ẍ = cos(t)/m.
}

GTEST_TEST(ParticlePlantTest, PartialTest) {
  Particle1dPlant<double> test_particle;

  auto context = test_particle.CreateDefaultContext();
  auto& state = context->get_mutable_continuous_state_vector();

  // Test the derivatives at an arbitrary time of 3 seconds.
  context->set_time(3.0);
  auto time = context->get_time();

  // Get mass from the particle.
  auto mass = test_particle.get_mass();

  // The state vector has the following assignments:
  // state[0] = x and state[1] = ẋ.
  int x_index = 0;
  int xDt_index = 1;

  // Set the state assuming the system has evolved over time.
  state.SetAtIndex(x_index, (-cos(time) + 1) / mass);  // x = (-cos(t) + 1)/m.
  state.SetAtIndex(xDt_index, sin(time) / mass);       // ẋ = sin(t)/m

  // Transmogrify the particle to autodiff.
  std::unique_ptr<Particle1dPlant<AutoDiffXd>> ad_test_particle =
      systems::System<double>::ToAutoDiffXd(test_particle);

  // Construct a new context based on AutoDiff.
  auto ad_context = ad_test_particle->CreateDefaultContext();
  ad_context->SetTimeStateAndParametersFrom(*context);
  auto ad_derivatives = ad_test_particle->AllocateTimeDerivatives();

  auto& ad_state = ad_context->get_mutable_continuous_state_vector();

  // Check that the transformed AutoDiff vector has held on to the value of the
  // initlialized double type and that the derivatives vector has been created.
  EXPECT_EQ(ad_state[0].value(), state[0]);
  EXPECT_EQ(ad_state[1].value(), state[1]);
  EXPECT_EQ(ad_state[0].derivatives().size(), 0);
  EXPECT_EQ(ad_state[1].derivatives().size(), 0);

  // Compute derivatives.
  ad_test_particle->CalcTimeDerivatives(*ad_context, ad_derivatives.get());

  const systems::VectorBase<AutoDiffXd>& new_derivatives_vector =
      ad_derivatives->get_vector();

  // Check derivative results against the analytical solution.
  EXPECT_EQ(new_derivatives_vector[0].value(),
            sin(time) / mass);  // ẋ = sin(t)/m.
  EXPECT_EQ(new_derivatives_vector[1].value(),
            cos(time) / mass);  // ẍ = cos(t)/m.
}

} // namespace
} // namespace particle1d
} // namespace examples
} // namespace drake

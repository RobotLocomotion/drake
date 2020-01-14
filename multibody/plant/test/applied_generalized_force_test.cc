#include <functional>
#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using multibody::Parser;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::VectorBase;
using systems::controllers::InverseDynamicsController;

namespace multibody {
namespace {

class MultibodyPlantGeneralizedAppliedForceTest
    : public ::testing::TestWithParam<double> {
 public:
  void SetUp() override {
    // Load two Iiwa models.
    const std::string full_name = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf");
    systems::DiagramBuilder<double> builder;
    const double dt = GetParam();
    plant_ = builder.AddSystem<MultibodyPlant<double>>(dt);

    // Add the model twice.
    auto iiwa1 = Parser(plant_).AddModelFromFile(full_name, "iiwa1");
    auto iiwa2 = Parser(plant_).AddModelFromFile(full_name, "iiwa2");
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0", iiwa1));
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0", iiwa2));
    plant_->Finalize();

    // Set feedback gains to zero - we won't need feedback control.
    const int nv = plant_->num_velocities();
    const VectorX<double> kp = VectorX<double>::Zero(nv);
    const VectorX<double> ki = VectorX<double>::Zero(nv);
    const VectorX<double> kd = VectorX<double>::Zero(nv);

    // Add the inverse dynamics controller.
    auto id_controller = builder.AddSystem<InverseDynamicsController<double>>(
        *plant_, kp, ki, kd, false /* reference acceleration is zero */);

    // Connect the ID controller to the MBP.
    builder.Connect(id_controller->get_output_port_control(),
                    plant_->get_applied_generalized_force_input_port());
    builder.Connect(plant_->get_state_output_port(),
                    id_controller->get_input_port_estimated_state());
    builder.Connect(plant_->get_state_output_port(),
                    id_controller->get_input_port_desired_state());

    // Plug the actuator inputs.
    auto u1_source = builder.AddSystem<ConstantVectorSource<double>>(
        VectorX<double>::Zero(plant_->num_velocities(iiwa1)));
    auto u2_source = builder.AddSystem<ConstantVectorSource<double>>(
        VectorX<double>::Zero(plant_->num_velocities(iiwa2)));
    builder.Connect(u1_source->get_output_port(),
                    plant_->get_actuation_input_port(iiwa1));
    builder.Connect(u2_source->get_output_port(),
                    plant_->get_actuation_input_port(iiwa2));

    diagram_ = builder.Build();

    // Create a context.
    context_ = diagram_->CreateDefaultContext();
  }

  // Gets a reference to the diagram.
  Diagram<double>& diagram() { return *diagram_; }

  // Gets a reference to the context.
  Context<double>& context() { return *context_; }

  // Gets a reference to the MultibodyPlant.
  MultibodyPlant<double>& plant() { return *plant_; }

 private:
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  MultibodyPlant<double>* plant_{nullptr};
};

// Verifies the applied generalized input force port by checking that the output
// of an inverse dynamics controller applied to two Iiwa's can be piped back
// into the plant and the correct derivatives obtained. Specifically, this test
// verifies that gravity compensation produces no acceleration (i.e., velocity
// derivatives should be zero).
TEST_P(MultibodyPlantGeneralizedAppliedForceTest,
       CheckGeneralizedAppliedForceInput) {
  auto& plant_context =
      diagram().GetMutableSubsystemContext(plant(), &context());

  // Put the robot into a configuration where it has nonzero potential
  // energy.
  const bool continuous_system = (GetParam() == 0.0);
  VectorBase<double>& plant_state =
      (continuous_system) ? plant_context.get_mutable_continuous_state_vector()
                          : plant_context.get_mutable_discrete_state_vector();
  for (int i = 0; i < plant().num_positions(); ++i)
    plant_state[0] = 0.1 * (i + 1);

  // Tolerance for equality (should be sufficient for the values near zero like
  // we expect).
  const double eps = 100 * std::numeric_limits<double>::epsilon();

  if (continuous_system) {
    // Compute time derivatives of velocity variables and ensure that they're
    // sufficiently near zero.
    auto derivatives = context().get_continuous_state().Clone();
    diagram().CalcTimeDerivatives(context(), derivatives.get());
    const VectorBase<double>& derivatives_vector = derivatives->get_vector();

    // Time-derivatives of velocities start immediately after positions in the
    // continuous state vector.
    for (int i = plant().num_positions(); i < derivatives->size(); ++i)
      EXPECT_NEAR(derivatives_vector[i], 0.0, eps);
  } else {
    // Compute discrete state updates and verify that velocity variables are
    // unchanged.
    auto& discrete_state_vector = context().get_discrete_state_vector();
    auto new_discrete_state = diagram().AllocateDiscreteVariables();
    const VectorBase<double>& new_discrete_state_vector =
        new_discrete_state->get_vector();
    diagram().CalcDiscreteVariableUpdates(context(), new_discrete_state.get());

    // Velocities start immediately after positions in the discrete state
    // vector.
    for (int i = plant().num_positions(); i < new_discrete_state_vector.size();
         ++i)
      EXPECT_NEAR(new_discrete_state_vector[i] - discrete_state_vector[i], 0.0,
                  eps);
  }
}

INSTANTIATE_TEST_SUITE_P(GravityCompensationTest,
                        MultibodyPlantGeneralizedAppliedForceTest,
                        ::testing::Values(0.0 /* continuous-time MBP */,
                                          1e-3 /* time stepping MBP */));

}  // namespace
}  // namespace multibody
}  // namespace drake

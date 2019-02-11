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
using systems::VectorBase;
using systems::controllers::InverseDynamicsController;

namespace multibody {
namespace {

// Verifies the applied generalized input force port by checking that the output
// of an inverse dynamics controller applied to two Iiwa's can be piped back
// into the plant and the correct derivatives obtained. Specifically, this test
// verifies that gravity compensation produces no acceleration (i.e., velocity
// derivatives should be zero).
GTEST_TEST(MultibodyPlantTest, CheckGeneralizedAppliedForceInput) {
  // Load two Iiwa models.
  const std::string full_name = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf");
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<MultibodyPlant<double>>();

  // Add the model twice.
  auto iiwa1 = Parser(plant).AddModelFromFile(full_name, "iiwa1");
  auto iiwa2 = Parser(plant).AddModelFromFile(full_name, "iiwa2");
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", iiwa1));
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", iiwa2));
  plant->AddForceElement<UniformGravityFieldElement>();
  plant->Finalize();

  // Set feedback gains to zero - we won't need feedback control.
  const int nv = plant->num_velocities();
  const VectorX<double> kp = VectorX<double>::Zero(nv);
  const VectorX<double> ki = VectorX<double>::Zero(nv);
  const VectorX<double> kd = VectorX<double>::Zero(nv);

  // Add the inverse dynamics controller.
  auto id_controller = builder.AddSystem<InverseDynamicsController<double>>(
      *plant, kp, ki, kd, false /* reference acceleration is zero */);

  // Connect the ID controller to the MBP.
  builder.Connect(
      id_controller->get_output_port_control(),
      plant->get_applied_generalized_force_input_port());
  builder.Connect(plant->get_continuous_state_output_port(),
                  id_controller->get_input_port_estimated_state());
  builder.Connect(plant->get_continuous_state_output_port(),
                  id_controller->get_input_port_desired_state());

  // Plug the actuator inputs.
  auto u1_source = builder.AddSystem<ConstantVectorSource<double>>(
      VectorX<double>::Zero(plant->num_velocities(iiwa1)));
  auto u2_source = builder.AddSystem<ConstantVectorSource<double>>(
      VectorX<double>::Zero(plant->num_velocities(iiwa2)));
  builder.Connect(u1_source->get_output_port(),
                  plant->get_actuation_input_port(iiwa1));
  builder.Connect(u2_source->get_output_port(),
                  plant->get_actuation_input_port(iiwa2));

  auto diagram = builder.Build();

  // Create a context.
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->GetMutableSubsystemContext(
      *plant, context.get());

  // Put the robot into a configuration where it has nonzero potential
  // energy.
  VectorBase<double>& plant_state =
    plant_context.get_mutable_continuous_state_vector();
  for (int i = 0; i < plant->num_positions(); ++i)
    plant_state[0] = 0.1 * (i + 1);

  // Compute time derivatives and ensure that they're sufficiently near zero.
  auto derivatives = context->get_continuous_state().Clone();
  diagram->CalcTimeDerivatives(*context, derivatives.get());
  const VectorBase<double>& derivatives_vector = derivatives->get_vector();
  const double eps = 100 * std::numeric_limits<double>::epsilon();
  for (int i = plant->num_positions(); i < derivatives->size(); ++i)
    EXPECT_NEAR(derivatives_vector[i], 0.0, eps);
}

}  // namespace
}  // namespace multibody
}  // namespace drake

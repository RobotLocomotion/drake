#include "drake/manipulation/scene_generation/simulate_plant_to_rest.h"

#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

SimulatePlantToRest::SimulatePlantToRest(
    std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant,
    std::unique_ptr<systems::LeafSystem<double>> visualizer)
    : plant_ptr_(scene_plant.get()),
      diagram_(GenerateDiagram(std::move(scene_plant), std::move(visualizer))) {
}

std::unique_ptr<systems::Diagram<double>> SimulatePlantToRest::GenerateDiagram(
    std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant,
    std::unique_ptr<systems::LeafSystem<double>> visualizer) {
  systems::DiagramBuilder<double> builder;

  // Transferring ownership of tree to the RigidBodyPlant.
  auto plant = builder.template AddSystem(std::move(scene_plant));
  plant->set_name("RBP");

  systems::CompliantMaterial default_material;
  default_material
      .set_youngs_modulus(5e6)  // Pa
      .set_dissipation(12)      // s/m
      .set_friction(1.2, 0.5);
  plant->set_default_compliant_material(default_material);

  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = 2e-4;  // m
  model_parameters.v_stiction_tolerance = 0.1;    // m/s
  plant->set_contact_model_parameters(model_parameters);

  if (visualizer) {
    auto visualizer_system = builder.template AddSystem(std::move(visualizer));
    visualizer_system->set_name("visualizer");

    builder.Connect(plant->get_output_port(0),
                    visualizer_system->get_input_port(0));
  }

  if (plant->get_num_actuators() > 0) {
    auto zero_input = builder.template AddSystem<systems::ConstantVectorSource>(
        Eigen::VectorXd::Zero(plant->get_num_actuators()));
    builder.Connect(zero_input->get_output_port(), plant->get_input_port(0));
  }

  return builder.Build();
}

VectorX<double> SimulatePlantToRest::Run(const VectorX<double>& q_ik,
                                         VectorX<double>* v_final,
                                         double v_threshold,
                                         double max_settling_time) {
  systems::Simulator<double> simulator(*diagram_);

  int num_positions = plant_ptr_->get_num_positions();
  int num_velocities = plant_ptr_->get_num_velocities();

  // Setting initial condition
  VectorX<double> x_initial =
      VectorX<double>::Zero(num_positions + num_velocities);

  VectorX<double> v = VectorX<double>::Zero(num_velocities);
  x_initial.head(num_positions) = q_ik;

  VectorX<double> x = x_initial;
  double step_time, step_delta;
  double max_step_size = 0.001;
  do {
    simulator.get_mutable_context()
        .get_mutable_continuous_state_vector()
        .SetFromVector(x_initial);
    simulator.Initialize();
    simulator.get_mutable_context().SetTime(0.0);

    simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
        *diagram_, 0.0001, &simulator.get_mutable_context());
    simulator.get_mutable_integrator().set_maximum_step_size(max_step_size);
    simulator.get_mutable_integrator().set_fixed_step_mode(true);
    step_time = 1.0;
    step_delta = 0.1;
    do {
      simulator.AdvanceTo(step_time);
      step_time += step_delta;
      x = simulator.get_context().get_continuous_state_vector().CopyToVector();
      v = x.tail(num_velocities);
    } while ((v.array() > v_threshold).any() && step_time <= max_settling_time);
    // If terminal velocities are nan, then half the step size and try again.
    if (isnan(v.array()).any()) {
      max_step_size /= 2.0;
      drake::log()->debug(
          "Simulation exploded. Halving max_time_step and restarting");
    }
  } while (isnan(v.array()).any());

  if (v_final != nullptr) {
    *v_final = v;
  }
  drake::log()->debug("In-Simulation time : {} sec", step_time);
  return x.head(num_positions);
}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake

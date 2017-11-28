/*! @file
A simple example of a rigid gripper attempting to hold a block. The gripper
has rigid geometry: two fingers at a fixed distance from each other. They
are positioned in a configuration *slightly* narrower than the box placed
between them.

This is a test to evaluate/debug the contact model.  This configuration
simplifies the test by defining a known penetration and eliminating all
controller-dependent variation.

This is an even simpler example of what is shown in schung_wsg_lift_test.
This eliminates the PID controller and ground contact.  At the end of the
simulation, the box should have slipped an amount less than the duration
length times v_stiction_tolerance (i.e., the highest allowable slip speed
during stiction).
*/

#include <algorithm>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

DEFINE_double(accuracy, 5e-5, "Sets the simulation accuracy");
DEFINE_double(us, 0.9, "The coefficient of static friction");
DEFINE_double(ud, 0.5, "The coefficient of dynamic friction");
DEFINE_double(youngs_modulus, 1e8, "The contact material Young's modulus (Pa)");
DEFINE_double(dissipation, 2.0, "The contact material dissipation (s/m)");
DEFINE_double(v_stiction_tolerance, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(contact_area, 1e-4,
              "The characteristic scale of contact area (m^2)");
DEFINE_double(sim_duration, 5, "Amount of time to simulate (s)");
DEFINE_bool(playback, true,
            "If true, simulation begins looping playback when complete");

namespace drake {
namespace examples {

using drake::systems::RungeKutta3Integrator;
using drake::systems::ContactResultsToLcmSystem;
using drake::systems::lcm::LcmPublisherSystem;

std::unique_ptr<RigidBodyTreed> BuildTestTree() {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();

  // Add the gripper.  Offset it slightly back and up so that we can
  // locate the box at the origin.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "gripper_pose_frame",
      &tree->world(), Eigen::Vector3d(0, -0.065, 0.05),
      Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/examples/contact_model/rigid_gripper.urdf"),
      multibody::joints::kFixed, gripper_frame, tree.get());

  // Add a box to grip.  Position it such that if there *were* a plane at z = 0,
  // the box would be sitting on it (this maintains parity with the
  // schunk_wsg_lift_test scenario).
  auto box_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box_offset",
      &tree->world(), Eigen::Vector3d(0, 0, 0.075), Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/multibody/models/box_small.urdf"),
      multibody::joints::kQuaternion, box_frame, tree.get());

  return tree;
}

int main() {
  systems::DiagramBuilder<double> builder;

  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(BuildTestTree());
  plant->set_name("plant");

  // Command-line specified contact parameters.
  std::cout << "Contact properties:\n";
  std::cout << "\tYoung's modulus:          " << FLAGS_youngs_modulus << "\n";
  std::cout << "\tDissipation:              " << FLAGS_dissipation << "\n";
  std::cout << "\tstatic friction:          " << FLAGS_us << "\n";
  std::cout << "\tdynamic friction:         " << FLAGS_ud << "\n";
  std::cout << "\tAllowed stiction speed:   " << FLAGS_v_stiction_tolerance
            << "\n";
  std::cout << "\tDissipation:              " << FLAGS_dissipation << "\n";

  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_area = FLAGS_contact_area;
  model_parameters.v_stiction_tolerance = FLAGS_v_stiction_tolerance;
  plant->set_contact_model_parameters(model_parameters);

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm, true);
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));

  // Enable contact force visualization.
  const ContactResultsToLcmSystem<double>& contact_viz =
  *builder.template AddSystem<ContactResultsToLcmSystem<double>>(
      plant->get_rigid_body_tree());
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant->contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port(0));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);

  systems::Context<double>& context = simulator.get_mutable_context();

  simulator.reset_integrator<RungeKutta3Integrator<double>>(*model, &context);
  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-4);
  simulator.get_mutable_integrator()->set_target_accuracy(FLAGS_accuracy);
  std::cout << "Variable-step integrator accuracy: " << FLAGS_accuracy << "\n";

  simulator.Initialize();

  // Print a time stamp update every tenth of a second.  This helps communicate
  // progress in the event that the integrator crawls to a very small timestep.
  const double kPrintPeriod = std::min(0.1, FLAGS_sim_duration);
  int step_count =
      static_cast<int>(std::ceil(FLAGS_sim_duration / kPrintPeriod));
  for (int i = 1; i <= step_count; ++i) {
    double t = context.get_time();
    std::cout << "time: " << t << "\n";
    simulator.StepTo(i * kPrintPeriod);
  }

  while (FLAGS_playback) viz_publisher->ReplayCachedSimulation();
  return 0;
}

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::main();
}

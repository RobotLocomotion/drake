/// @file
///
/// This demo sets up a simple passive dynamics simulation of the Kinova Jaco
/// arm. The robot is initialized with an (arbitrary) joint space pose, and is
/// simulated with zero torques at the joints.

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
  
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/geometry/geometry_visualization.h"
// #include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {


namespace examples {
namespace allegro_hand {
namespace {

using drake::multibody::multibody_plant::MultibodyPlant;
using drake::systems::rendering::PoseBundleToDrawMessage;
// using drake::multibody::RevoluteJoint;

DEFINE_double(simulation_time, 1e-2, "Number of seconds to simulate");

DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates of period 'max_time_step'."
    "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(max_time_step, 1.0e-3,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");

// Contact parameters
DEFINE_double(penetration_allowance, 1.0e-2,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-2,
              "The maximum slipping speed allowed during stiction. [m/s]");

// Integration parameters:
DEFINE_string(integration_scheme, "semi_explicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");

DEFINE_bool(add_gravity, false, "Whether adding gravity in the simulation");

DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");
DEFINE_double(target_realtime_rate, 1e-2,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(constant_load_force, 0, "the constant load on all the joint");


int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph = 
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant =
      FLAGS_time_stepping ?
      *builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step) : /*discrete*/
      *builder.AddSystem<MultibodyPlant>();   /* continuous system */
  std::string full_name =
      FindResourceOrThrow("drake/manipulation/models/allegro_hand_description/"
                          // "sdf/allegro_hand_description_right_full.sdf");
                          // "sdf/allegro_finger_1.sdf");
                          "sdf/allegro_finger_saperate.sdf");
  multibody::parsing::AddModelFromSdfFile(
                          full_name, &plant, &scene_graph);


  // optional: adding gravity -- the gripper could start free falling
  if (FLAGS_add_gravity)
    plant.AddForceElement<multibody::UniformGravityFieldElement>(
        -9.81 * Eigen::Vector3d::UnitZ());

  // Now the model is complete.
  plant.Finalize(&scene_graph);

  PRINT_VAR(plant.model().num_velocities());
  PRINT_VAR(plant.model().num_positions());
  PRINT_VAR(plant.num_actuators());
  PRINT_VAR(plant.num_actuated_dofs());

  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // If the user specifies a time step, we use that, otherwise estimate a
  // maximum time step based on the compliance of the contact model.
  // The maximum time step is estimated to resolve this time scale with at
  // least 30 time steps. Usually this is a good starting point for fixed step
  // size integrators to be stable.
  const double max_time_step =
      FLAGS_max_time_step > 0 ? FLAGS_max_time_step :
      plant.get_contact_penalty_method_time_scale() / 30;

  // Print maximum time step and the time scale introduced by the compliance in
  // the contact model as a reference to the user.
  fmt::print("Maximum time step = {:10.6f} s\n", max_time_step);
  fmt::print("Compliance time scale = {:10.6f} s\n",
             plant.get_contact_penalty_method_time_scale());

  // DRAKE_DEMAND(plant.num_actuators() == 16);
  // DRAKE_DEMAND(plant.num_actuated_dofs() == 16);

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  const systems::rendering::PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  systems::lcm::LcmPublisherSystem& publisher =
      *builder.template AddSystem<systems::lcm::LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<systems::lcm::Serializer<lcmt_viewer_draw>>(), &lcm);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Publish contact results for visualization.
  #if 1
  const auto& contact_results_to_lcm =
      *builder.AddSystem<multibody::multibody_plant::ContactResultsToLcmSystem>
      (plant);
  const auto& contact_results_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>
      ("CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_results_to_lcm.get_input_port(0));
  builder.Connect(contact_results_to_lcm.get_output_port(0),
                  contact_results_publisher.get_input_port());
  #endif

  // zero force input
  VectorX<double> constant_load_value = VectorX<double>::Ones(
      plant.model().num_actuators()) * FLAGS_constant_load_force;
  auto constant_source =
     builder.AddSystem<systems::ConstantVectorSource<double>>(
      constant_load_value);
  constant_source->set_name("constant_source");
  builder.Connect(constant_source->get_output_port(), 
                  plant.get_actuation_input_port());

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
 

  // intialization state?
  std::cout<<"Initialize hand state \n";



  //PRINT_VAR(plant_context.get_discrete_state(0).CopyToVector().transpose());

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  const multibody::RevoluteJoint<double>& joint_finger_1_root =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_1");
  joint_finger_1_root.set_angle(&plant_context, 0.5);
  // const multibody::RevoluteJoint<double>& joint_finger_1_tip =
  //     plant.GetJointByName<multibody::RevoluteJoint>("joint_3");
  // joint_finger_1_tip.set_angle(&plant_context, 0.5);
  //   const multibody::RevoluteJoint<double>& joint_finger_1_middle =
  //     plant.GetJointByName<multibody::RevoluteJoint>("joint_2");
  // joint_finger_1_middle.set_angle(&plant_context, 0.5);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);


  // The error controlled integrators might need to take very small time steps
  // to compute a solution to the desired accuracy. Therefore, to visualize
  // these very short transients, we publish every time step.
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 1;


}  // int main

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::allegro_hand::DoMain();
}

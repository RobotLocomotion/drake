/// @file
///
/// This file set up a simulation environment of an allegro hand and an object.
/// The system is designed for position control of the hand, with a PID
/// controller to control the output torque. The system communicate with the
/// external program through LCM system, with a publisher to publish the
/// current state of the hand, and a subscriber to read the posiiton commands
/// of the finger joints.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::MultibodyPlant;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");
DEFINE_double(max_time_step, 1.5e-4,
              "Simulation time step used for intergrator.");
DEFINE_bool(add_gravity, false,
            "Whether adding gravity (9.81 m/s^2) in the simulation");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'radau1', 'implicit_euler' (ec), 'velocity_implicit_euler' (ec), 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");

DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");
DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates of period 'max_time_step'."
    "If 'false', the plant is modeled as a continuous system.");
DEFINE_bool(error_control, false, "If 'true', integrator uses error control.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");
  double max_time_step = 0;
  if (FLAGS_time_stepping)
    max_time_step = FLAGS_max_time_step;
  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(max_time_step);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string hand_model_path;
  if (FLAGS_use_right_hand)
    hand_model_path = FindResourceOrThrow(
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_right.sdf");
  else
    hand_model_path = FindResourceOrThrow(
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_left.sdf");

  const std::string object_model_path = FindResourceOrThrow(
      "drake/examples/allegro_hand/joint_control/simple_mug.sdf");
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(hand_model_path);
  parser.AddModelFromFile(object_model_path);

  // Weld the hand to the world frame
  const auto& joint_hand_root = plant.GetBodyByName("hand_root");
  plant.AddJoint<multibody::WeldJoint>("weld_hand", plant.world_body(),
                                       std::nullopt,
                                       joint_hand_root,
                                       std::nullopt,
                                       RigidTransformd::Identity());

  if (!FLAGS_add_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Finished building the plant
  plant.Finalize();

  // Visualization
  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  // Publish contact results for visualization.
  multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant, lcm);

  // PID controller for position control of the finger joints
  VectorX<double> kp, kd, ki;
  MatrixX<double> Sx, Sy;
  GetControlPortMapping(plant, &Sx, &Sy);
  SetPositionControlledGains(&kp, &ki, &kd);
  auto& hand_controller = *builder.AddSystem<
      systems::controllers::PidController>(Sx, Sy, kp, ki, kd);
  builder.Connect(plant.get_state_output_port(),
                  hand_controller.get_input_port_estimated_state());
  builder.Connect(hand_controller.get_output_port_control(),
                  plant.get_actuation_input_port());

  // Create an output port of the continuous state from the plant that only
  // output the status of the hand finger joints related DOFs, and put them in
  // the pre-defined order that is easy for understanding.
  const auto& hand_status_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sx);
  builder.Connect(plant.get_state_output_port(),
                  hand_status_converter.get_input_port());
  const auto& hand_output_torque_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sy);
  builder.Connect(hand_controller.get_output_port_control(),
                  hand_output_torque_converter.get_input_port());

  // Create the command subscriber and status publisher for the hand.
  auto& hand_command_sub = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_allegro_command>(
          "ALLEGRO_COMMAND", lcm));
  hand_command_sub.set_name("hand_command_subscriber");
  auto& hand_command_receiver =
      *builder.AddSystem<AllegroCommandReceiver>(kAllegroNumJoints);
  hand_command_receiver.set_name("hand_command_receiver");
  auto& hand_status_pub = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_allegro_status>(
          "ALLEGRO_STATUS", lcm, kLcmStatusPeriod /* publish period */));
  hand_status_pub.set_name("hand_status_publisher");
  auto& status_sender =
      *builder.AddSystem<AllegroStatusSender>(kAllegroNumJoints);
  status_sender.set_name("status_sender");

  builder.Connect(hand_command_sub.get_output_port(),
                  hand_command_receiver.get_input_port(0));
  builder.Connect(hand_command_receiver.get_commanded_state_output_port(),
                  hand_controller.get_input_port_desired_state());
  builder.Connect(hand_status_converter.get_output_port(),
                  status_sender.get_state_input_port());
  builder.Connect(hand_command_receiver.get_output_port(0),
                  status_sender.get_command_input_port());
  builder.Connect(hand_output_torque_converter.get_output_port(),
                  status_sender.get_commanded_torque_input_port());
  builder.Connect(status_sender.get_output_port(0),
                  hand_status_pub.get_input_port());

  // Now the model is complete.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  geometry::DispatchLoadMessage(scene_graph, lcm);
  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  // Set the position of object
  const multibody::Body<double>& mug = plant.GetBodyByName("main_body");
  const multibody::Body<double>& hand = plant.GetBodyByName("hand_root");
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize the mug pose to be right in the middle between the fingers.
  const Eigen::Vector3d& p_WHand =
      plant.EvalBodyPoseInWorld(plant_context, hand).translation();
  RigidTransformd X_WM(
      RollPitchYawd(M_PI / 2, 0, 0),
      p_WHand + Eigen::Vector3d(0.095, 0.062, 0.095));
  plant.SetFreeBodyPose(&plant_context, mug, X_WM);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  // for VIE statistics
  bool vie = false;
  bool implicit = false;
  systems::VelocityImplicitEulerIntegrator<double>* vie_integrator{nullptr};
  systems::ImplicitIntegrator<double>* implicit_integrator{nullptr};
  
  if (FLAGS_integration_scheme == "implicit_euler") {
    implicit_integrator =
        simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    implicit = true;
    integrator = implicit_integrator;
    integrator->set_target_accuracy(FLAGS_accuracy);
    integrator->set_fixed_step_mode(!FLAGS_error_control);
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "velocity_implicit_euler") {
    vie_integrator =
        simulator.reset_integrator<systems::VelocityImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    vie = true;
    implicit_integrator = vie_integrator;
    implicit = true;
    integrator = implicit_integrator;
    integrator->set_target_accuracy(FLAGS_accuracy);
    integrator->set_fixed_step_mode(!FLAGS_error_control);
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
  integrator->set_maximum_step_size(FLAGS_max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);
  
  integrator->set_fixed_step_mode(!FLAGS_error_control);

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // set the initial command for the hand
  hand_command_receiver.set_initial_position(
      &diagram->GetMutableSubsystemContext(hand_command_receiver,
                                           &simulator.get_mutable_context()),
      VectorX<double>::Zero(plant.num_actuators()));

  simulator.AdvanceTo(FLAGS_simulation_time);


  if (FLAGS_time_stepping) {
    fmt::print("Used time stepping with dt={}\n", FLAGS_max_time_step);
    fmt::print("Number of time steps taken = {:d}\n",
               simulator.get_num_steps_taken());
  } else {
    fmt::print("Stats for integrator {}:\n", FLAGS_integration_scheme);
    fmt::print("Number of time steps taken = {:d}\n",
               vie ? vie_integrator->get_vie_num_steps_taken() : integrator->get_num_steps_taken());
    if (!integrator->get_fixed_step_mode()) {
      fmt::print("Initial time step taken = {:10.6g} s\n",
                 vie ? vie_integrator->get_vie_actual_initial_step_size_taken() : 
                 integrator->get_actual_initial_step_size_taken());
      fmt::print("Largest time step taken = {:10.6g} s\n",
                 vie ? vie_integrator->get_vie_largest_step_size_taken() : 
                 integrator->get_largest_step_size_taken());
      fmt::print("Smallest adapted step size = {:10.6g} s\n",
                 vie ? vie_integrator->get_vie_smallest_adapted_step_size_taken() :
                 integrator->get_smallest_adapted_step_size_taken());
      fmt::print("Number of steps shrunk due to error control = {:d}\n",
                 integrator->get_num_step_shrinkages_from_error_control());
      fmt::print("Number of steps shrunk due to convergence-based failure = {:d}\n",
                 integrator->get_num_step_shrinkages_from_substep_failures());
      fmt::print("Number of convergence-based step failures (should match) = {:d}\n",
                 integrator->get_num_substep_failures());
    }
    if(implicit)
    {
      if(vie)
        fmt::print("Implicit Integrator Statistics (total, half-size-steps):\n");
      else
        fmt::print("Implicit Integrator Statistics (total, error estimator):\n");
      fmt::print("Number of Derivative Evaluations = {:d}, {:d} \n",
                 implicit_integrator->get_num_derivative_evaluations(),
                 implicit_integrator->get_num_error_estimator_derivative_evaluations());
      fmt::print("Number of Jacobian Computations = {:d}, {:d} \n",
                 implicit_integrator->get_num_jacobian_evaluations(),
                 implicit_integrator->get_num_error_estimator_jacobian_evaluations());
      fmt::print("Number of Derivative Evaluations for Jacobians = {:d}, {:d} \n",
                 implicit_integrator->get_num_derivative_evaluations_for_jacobian(),
                 implicit_integrator->get_num_error_estimator_derivative_evaluations_for_jacobian());
      fmt::print("Number of Iteration Matrix Factorizations = {:d}, {:d} \n",
                 implicit_integrator->get_num_iteration_matrix_factorizations(),
                 implicit_integrator->get_num_error_estimator_iteration_matrix_factorizations());
      fmt::print("Number of Newton-Raphson Iterations = {:d}, {:d} \n",
                 implicit_integrator->get_num_newton_raphson_iterations(),
                 implicit_integrator->get_num_error_estimator_newton_raphson_iterations());
      fmt::print("Number of Newton-Raphson Iterations That Lead to Failure = {:d}, {:d} \n",
                 implicit_integrator->get_num_newton_raphson_iterations_that_end_in_failure(),
                 implicit_integrator->get_num_error_estimator_newton_raphson_iterations_that_end_in_failure());
      fmt::print("Number of Newton-Raphson Failures = {:d}, {:d} \n",
                 implicit_integrator->get_num_newton_raphson_failures(),
                 implicit_integrator->get_num_error_estimator_newton_raphson_failures());
    }
  }
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::allegro_hand::DoMain();
  return 0;
}

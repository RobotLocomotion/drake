/// @file
///
/// This demo sets up a simple dynamic simulation for the Allegro hand using
/// the multi-body library. The joint torques are constant values that can be
/// set manually, and is the same for all the joints. This demo also allows to
/// specify whether the right or left hand is simulated.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {


namespace examples {
namespace allegro_hand {
namespace {

using drake::multibody::multibody_plant::MultibodyPlant;

DEFINE_double(constant_load, 0, "the constant load on each joint, Unit [N*m]."
              "Suggested load is in the order of 0.01. When input value equals"
              "to 0 (default), the program runs a passive simulation. ");

DEFINE_double(simulation_time, 5, 
              "Desired duration of the simulation in seconds");

DEFINE_string(test_hand, "right", "Which hand to model: 'left' or 'right'");

DEFINE_double(max_time_step, 1.0e-4, "Simulation time step in seconds.");

DEFINE_bool(add_gravity, true, 
            "Whether adding gravity (9.81 m/s^2) in the simulation");

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph = 
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>
                                  (FLAGS_max_time_step);
  const std::string full_name = FindResourceOrThrow("drake/manipulation/models"
                  "/allegro_hand_description/sdf/allegro_hand_description_" 
                  + FLAGS_test_hand + ".sdf");
  multibody::parsing::AddModelFromSdfFile(
                          full_name, &plant, &scene_graph);

  // Weld the hand to the world frame
  const auto& joint_hand_root = plant.GetBodyByName("hand_root");
  plant.AddJoint<multibody::WeldJoint>( "weld_hand", plant.world_body(), {}, 
      joint_hand_root, {}, Isometry3<double>::Identity());

  // Add gravity, if needed
  if (FLAGS_add_gravity)
    plant.AddForceElement<multibody::UniformGravityFieldElement>(
        -9.81 * Eigen::Vector3d::UnitZ());

  // Now the model is complete.
  plant.Finalize(&scene_graph);

  DRAKE_DEMAND(plant.num_actuators() == 16);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 16);

  std::cout<<"model complete"<<std::endl;

  // Visualization
  lcm::DrakeLcm lcm;
  geometry::ConnectVisualization(scene_graph, &builder, &lcm);
  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

    std::cout<<"Visualization complete"<<std::endl;

  // constant force input
  VectorX<double> constant_load_value = VectorX<double>::Ones(
      plant.model().num_actuators()) * FLAGS_constant_load;
  auto constant_source =
     builder.AddSystem<systems::ConstantVectorSource<double>>(
      constant_load_value);
  constant_source->set_name("constant_source");
  builder.Connect(constant_source->get_output_port(), 
                  plant.get_actuation_input_port());

    std::cout<<"contact complete"<<std::endl;

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
    std::cout<<"diagram complete"<<std::endl;
  geometry::DispatchLoadMessage(scene_graph, &lcm);

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get()); 

  // Initialize joint angle. 3 joints on the index, middle and end fingers
  // are set to some random values.
  const multibody::RevoluteJoint<double>& joint_finger_1_root =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_1");
  joint_finger_1_root.set_angle(&plant_context, 0.5);
  const multibody::RevoluteJoint<double>& joint_finger_2_middle =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_6");
  joint_finger_2_middle.set_angle(&plant_context, -0.1);
  const multibody::RevoluteJoint<double>& joint_finger_3_tip =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_11");
  joint_finger_3_tip.set_angle(&plant_context, 0.5);

    std::cout<<"configuration complete"<<std::endl;

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
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
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::allegro_hand::DoMain();
}

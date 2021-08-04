/// @file
///
/// This demo sets up a simple dynamic simulation for the Allegro hand using
/// the multi-body library. A single, constant torque is applied to all joints
/// and defined by a command-line parameter. This demo also allows to specify
/// whether the right or left hand is simulated.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::multibody::MultibodyPlant;

DEFINE_double(constant_load, 0, "the constant load on each joint, Unit [Nm]."
              "Suggested load is in the order of 0.01 Nm. When input value"
              "equals to 0 (default), the program runs a passive simulation.");

DEFINE_double(simulation_time, 5,
              "Desired duration of the simulation in seconds");

DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");

DEFINE_double(max_time_step, 1.0e-4,
              "Simulation time step used for integrator.");

DEFINE_bool(add_gravity, true, "Indicator for whether terrestrial gravity"
                                " (9.81 m/s²) is included or not.");

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>
                                  (FLAGS_max_time_step);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string full_name;
  if (FLAGS_use_right_hand)
    full_name = FindResourceOrThrow("drake/manipulation/models/"
      "allegro_hand_description/sdf/allegro_hand_description_right.sdf");
  else
    full_name = FindResourceOrThrow("drake/manipulation/models/"
      "allegro_hand_description/sdf/allegro_hand_description_left.sdf");

  multibody::Parser(&plant).AddModelFromFile(full_name);

  // Weld the hand to the world frame
  const auto& joint_hand_root = plant.GetBodyByName("hand_root");
  plant.AddJoint<multibody::WeldJoint>("weld_hand", plant.world_body(),
      std::nullopt, joint_hand_root, std::nullopt,
      math::RigidTransformd::Identity());

  if (!FLAGS_add_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Now the model is complete.
  plant.Finalize();

  DRAKE_DEMAND(plant.num_actuators() == 16);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 16);

  // constant force input
  VectorX<double> constant_load_value = VectorX<double>::Ones(
      plant.num_actuators()) * FLAGS_constant_load;
  auto constant_source =
     builder.AddSystem<systems::ConstantVectorSource<double>>(
      constant_load_value);
  constant_source->set_name("constant_source");
  builder.Connect(constant_source->get_output_port(),
                  plant.get_actuation_input_port());

  DRAKE_DEMAND(plant.get_source_id().has_value());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize joint angle. 3 joints on the index, middle and ring fingers
  // are set to some arbitrary values.
  const multibody::RevoluteJoint<double>& joint_finger_1_root =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_1");
  joint_finger_1_root.set_angle(&plant_context, 0.5);
  const multibody::RevoluteJoint<double>& joint_finger_2_middle =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_6");
  joint_finger_2_middle.set_angle(&plant_context, -0.1);
  const multibody::RevoluteJoint<double>& joint_finger_3_tip =
      plant.GetJointByName<multibody::RevoluteJoint>("joint_11");
  joint_finger_3_tip.set_angle(&plant_context, 0.5);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
}

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

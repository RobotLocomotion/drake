// Implements a simulation of the Drake-compatible description of the
// PR2 robot with two tables and an object for gripping. 

#include <gflags/gflags.h>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace pr2 {

// TODO: wait for pull request to change this to a more general directory.
using kuka_iiwa_arm::ModelInstanceInfo;
using kuka_iiwa_arm::WorldSimTreeBuilder;
using kuka_iiwa_arm::RobotPlanInterpolator;

std::unique_ptr<RigidBodyTree<double>> build_world_tree(
    std::vector<ModelInstanceInfo<double>>* world_info_,
    std::vector<std::string> names, std::vector<std::string> description_paths,
    std::vector<Eigen::Vector3d> poses_xyz,
    std::vector<Eigen::Vector3d> poses_rpy, std::vector<bool> fixed) {
  auto tree_builder_ = std::make_unique<WorldSimTreeBuilder<double>>();
  for (int index = 0; index < (int)names.size(); index++) {
    tree_builder_->StoreModel(names[index], description_paths[index]);
  }
  for (int index = 0; index < (int)names.size(); index++) {
    if (fixed[index]) {
      int object_id = tree_builder_->AddFixedModelInstance(
          names[index], poses_xyz[index], poses_rpy[index]);
      world_info_->push_back(
          tree_builder_->get_model_info_for_instance(object_id));
    } else {
      int object_id = tree_builder_->AddFloatingModelInstance(
          names[index], poses_xyz[index], poses_rpy[index]);
      world_info_->push_back(
          tree_builder_->get_model_info_for_instance(object_id));
    }
  }

  tree_builder_->AddGround();
  return tree_builder_->Build();
}

int DoMain() {
  // Declare the diagram builder and lcm.
  systems::DiagramBuilder<double> diagram_builder;
  drake::lcm::DrakeLcm lcm;

  // Construct the tree for the PR2, a soda box, and two tables.
  std::vector<ModelInstanceInfo<double>> world_info_;

  std::vector<std::string> names;
  names.push_back("pr2");
  names.push_back("soda");
  names.push_back("table1");
  names.push_back("table2");

  std::vector<std::string> description_paths;
  const auto pr2_urdf_path = "drake/examples/pr2/models/pr2_description/urdf/pr2_simplified.urdf";
  description_paths.push_back(pr2_urdf_path);
  description_paths.push_back("drake/examples/pr2/models/objects/soda.urdf");
  description_paths.push_back("drake/examples/pr2/models/objects/table.sdf");
  description_paths.push_back("drake/examples/pr2/models/objects/table.sdf");

  std::vector<Eigen::Vector3d> initial_poses_xyz;
  Eigen::Vector3d initial_pr2_pose_xyz;
  initial_pr2_pose_xyz << 0, 0, 0;
  initial_poses_xyz.push_back(initial_pr2_pose_xyz);
  Eigen::Vector3d initial_soda_pose_xyz;
  initial_soda_pose_xyz << 1.8, 0.3, 0.83;
  initial_poses_xyz.push_back(initial_soda_pose_xyz);
  Eigen::Vector3d initial_table1_pose_xyz;
  initial_table1_pose_xyz << 2, 0, 0.38225;
  initial_poses_xyz.push_back(initial_table1_pose_xyz);
  Eigen::Vector3d initial_table2_pose_xyz;
  initial_table2_pose_xyz << 0, 2, 0.38225;
  initial_poses_xyz.push_back(initial_table2_pose_xyz);

  std::vector<Eigen::Vector3d> initial_poses_rpy;
  Eigen::Vector3d initial_pr2_pose_rpy;
  initial_pr2_pose_rpy << 0, 0, 0;
  initial_poses_rpy.push_back(initial_pr2_pose_rpy);
  Eigen::Vector3d initial_soda_pose_rpy;
  initial_soda_pose_rpy << 0, 0, -0.3;
  initial_poses_rpy.push_back(initial_soda_pose_rpy);
  Eigen::Vector3d initial_table1_pose_rpy;
  initial_table1_pose_rpy << 0, 0, 0;
  initial_poses_rpy.push_back(initial_table1_pose_rpy);
  Eigen::Vector3d initial_table2_pose_rpy;
  initial_table2_pose_rpy << 0, 0, 0;
  initial_poses_rpy.push_back(initial_table2_pose_rpy);
  
  std::vector<bool> fixed;
  fixed.push_back(true);
  fixed.push_back(false);
  fixed.push_back(true);
  fixed.push_back(true);

  auto tree_ = build_world_tree(&world_info_, names, description_paths, initial_poses_xyz, initial_poses_rpy, fixed);
  const auto pr2_instance_id = world_info_[0].instance_id;

  // We expect the number of actuators from the PR2 (and from the whole world) to be 28.
  DRAKE_ASSERT(tree_->get_num_actuators() == 28);
  const int num_actuators = 28;

  // Remove the PR2's collision groups that the PR2 doesn't need to grip with, to speed up the simulation.
  auto filter = [&](const std::string& group_name){
    return group_name == "non_gripper";
  };
  tree_->removeCollisionGroupsIf(filter);

  // Set the plant for the PR2.
  auto plant_ = diagram_builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_));
  plant_->set_name("plant_");

  // Add a plan reciever and interpolator to feed into the PR2's controller.
  auto plan_receiver_ = diagram_builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<robotlocomotion::robot_plan_t>(
          "PR2_PLAN", &lcm));
  plan_receiver_->set_name("plan_receiver");

  auto command_injector_ = diagram_builder.AddSystem<RobotPlanInterpolator>(
      drake::FindResourceOrThrow(pr2_urdf_path));
  command_injector_->set_name("command_injector");

  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      command_injector_->get_state_input_port());
  diagram_builder.Connect(plan_receiver_->get_output_port(0),
                           command_injector_->get_plan_input_port());

  // Add a PID controller for the PR2. 
  VectorX<double> kp(num_actuators);
  kp << 1000, 1000, 2500, 400000, 500, 500, 2000, 2000, 1000, 1000, 200, 200,
      50, 25, 25, 25, 25, 2000, 2000, 1000, 1000, 200, 200, 50, 25, 25, 25, 25;
  VectorX<double> ki(num_actuators);
  ki << 0, 0, 5, 20000, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0, 0, 0, 0, 5, 5, 5,
      5, 5, 5, 5, 0, 0, 0, 0;
  VectorX<double> kd(num_actuators);
  kd << 1820, 1820, 750, 0.7, 0.7, 0.7, 7, 5, 0.7, 0.7, 0.2, 0.7, 0.1, 0.1, 0.1, 0.1, 0.1, 7, 5, 0.7, 0.7,
      0.2, 0.7, 0.1, 0.1, 0.1, 0.1, 0.1;
  auto Binv = plant_->get_rigid_body_tree()
                  .B.block(0, 0, num_actuators, num_actuators)
                  .inverse();
  auto controller_ = diagram_builder.AddSystem<systems::controllers::PidController<double>>(
      std::make_unique<systems::controllers::PidController<double>>(
          MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), Binv, kp, ki,
          kd));

  diagram_builder.Connect(command_injector_->get_state_output_port(),
                           controller_->get_input_port_desired_state());
  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      controller_->get_input_port_estimated_state());
  diagram_builder.Connect(
      controller_->get_output_port_control(),
      plant_->model_instance_actuator_command_input_port(pr2_instance_id));

  // Add a visualizer.
  systems::DrakeVisualizer& visualizer_publisher =
      *diagram_builder.template AddSystem<systems::DrakeVisualizer>(
          plant_->get_rigid_body_tree(), &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  diagram_builder.Connect(plant_->state_output_port(),
                          visualizer_publisher.get_input_port(0));

  // Set contact parameters that support gripping.
  const double kStaticFriction = 1;
  const double kDynamicFriction = 5e-1;
  const double kStictionSlipTolerance = 1e-3;
  plant_->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                          kStictionSlipTolerance);

  const double kStiffness = 1000;
  const double kDissipation = 100;
  plant_->set_normal_contact_parameters(kStiffness, kDissipation);

  // Create the simulator.
  std::unique_ptr<systems::Diagram<double>> diagram_ = diagram_builder.Build();
  systems::Simulator<double> simulator(*diagram_);

  // Reset the integrator with parameters that support stable gripping, given
  // the contact parameters.
  auto context = simulator.get_mutable_context();
  const double max_step_size = 1e-4;
  simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      *diagram_, max_step_size, context);

  // Declare the initial pr2 state
  Eigen::VectorXd initial_pr2_state(num_actuators*2);
  initial_pr2_state << 0, 0, 0, 0.3, 0, 0, -1.14, 1.11, -1.40, -2.11,
      -1.33, -1.12, 2.19, 0.2, 0.2, 0.2, 0.2, 2.1, 1.29, 0 - 0.15, 0, -0.1, 0,
      0.2, 0.2, 0.2, 0.2, VectorX<double>::Zero(num_actuators);

  // Set the initial joint positions.
  for (int index = 0; index < num_actuators; index++) {
    plant_->set_position(context, index,
                         initial_pr2_state[index]);
  }

  // Set the initial joint velocities.
  for (int index = 0; index < num_actuators; index++) {
    plant_->set_velocity(context, index,
                         initial_pr2_state[index+num_actuators]);
  }

  // Initialize the robot plan interpolator.
  auto& plan_source_context = diagram_->GetMutableSubsystemContext(
      *command_injector_, context);
  command_injector_->Initialize(plan_source_context.get_time(),
                               Eigen::VectorBlock<VectorX<double>, num_actuators>(initial_pr2_state, 0),
                               plan_source_context.get_mutable_state());

  // Start the simulation.
  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pr2::DoMain();
}

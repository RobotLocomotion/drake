#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/common/find_resource.h"
#include "drake/examples/pr2/src/utils/robot_state_lcm.h"
#include "drake/examples/pr2/src/utils/plan_status_lcm.h"
#include "drake/examples/pr2/src/utils/world_sim_tree_builder.h"
#include "drake/examples/pr2/src/utils/robot_plan_interpolator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_robot_state.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"


namespace drake {
namespace examples {
namespace pr2 {

std::unique_ptr<RigidBodyTree<double>> build_world_tree(
    std::vector<ModelInstanceInfo<double>>* world_info,
    std::vector<std::string> names, std::vector<std::string> description_paths,
    std::vector<Eigen::Vector3d> poses_xyz,
    std::vector<Eigen::Vector3d> poses_rpy, std::vector<bool> fixed) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  for (int index = 0; index < (int)names.size(); index++) {
    tree_builder->StoreModel(names[index], description_paths[index]);
  }
  world_info->clear();
  for (int index = 0; index < (int)names.size(); index++) {
    if (fixed[index]) {
      int object_id = tree_builder->AddFixedModelInstance(
          names[index], poses_xyz[index], poses_rpy[index]);
      world_info->push_back(
          tree_builder->get_model_info_for_instance(object_id));
    } else {
      int object_id = tree_builder->AddFloatingModelInstance(
          names[index], poses_xyz[index], poses_rpy[index]);
      world_info->push_back(
          tree_builder->get_model_info_for_instance(object_id));
    }
  }

  tree_builder->AddGround();
  return tree_builder->Build();
}

void main(int argc, char* argv[]) {

   // Declare diagram builder and lcm
  systems::DiagramBuilder<double> diagram_builder;
  drake::lcm::DrakeLcm lcm;

  // Declare vector with initial robot joint positions
  Eigen::VectorXd initial_robot_joint_positions(24);
  initial_robot_joint_positions << 0., 0., 0., 0.3, 0., 0., -1.14, 1.11, -1.40, -2.11, -1.33, -1.12, 2.19, 0.49, 0.49, 2.1, 1.29, 0., -0.15, 0., -0.1, 0., 0.15, 0.15;


  // Construct the world tree, turn it into a plant, and add it to our diagram.
  std::vector<ModelInstanceInfo<double>> world_info;
  std::vector<std::string> names;
  names.push_back("pr2");
  names.push_back("table1");
  names.push_back("table2");
  names.push_back("soda");
  
  std::vector<std::string> description_paths;
  description_paths.push_back("drake/examples/pr2/pr2_description/urdf/pr2_with_joints_for_base_movement_and_limited_gripper_movement_no_collisions_except_grippers.urdf");
  description_paths.push_back("drake/examples/pr2/objects/drake_table.sdf");
  description_paths.push_back("drake/examples/pr2/objects/drake_table.sdf");
  description_paths.push_back("drake/examples/pr2/objects/drake_soda.urdf");

  std::vector<Eigen::Vector3d> initial_poses_xyz;
  Eigen::Vector3d pr2_xyz;
  pr2_xyz << 0, 0, 0;
  Eigen::Vector3d table1_xyz;
  table1_xyz << 2, 0, 0.38225;
  Eigen::Vector3d table2_xyz;
  table2_xyz << 0, 2, 0.38225;
  Eigen::Vector3d soda_xyz;
  soda_xyz << 1.8, 0.3, 0.83;
  initial_poses_xyz.push_back(pr2_xyz);
  initial_poses_xyz.push_back(table1_xyz);
  initial_poses_xyz.push_back(table2_xyz);
  initial_poses_xyz.push_back(soda_xyz);

  std::vector<Eigen::Vector3d> initial_poses_rpy;
  Eigen::Vector3d pr2_rpy;
  pr2_rpy << 0, 0, 0;
  Eigen::Vector3d table1_rpy;
  table1_rpy << 0, 0, 0;
  Eigen::Vector3d table2_rpy;
  table2_rpy << 0, 0, 0;
  Eigen::Vector3d soda_rpy;
  soda_rpy << 0, 0, 0;
  initial_poses_rpy.push_back(pr2_rpy);
  initial_poses_rpy.push_back(table1_rpy);
  initial_poses_rpy.push_back(table2_rpy);
  initial_poses_rpy.push_back(soda_rpy);

  std::vector<bool> fixed;
  fixed.push_back(true); 
  fixed.push_back(true);
  fixed.push_back(true);
  fixed.push_back(false);
  
  auto plant_ = diagram_builder.AddSystem<systems::RigidBodyPlant<double>>(build_world_tree(&world_info, names, description_paths, initial_poses_xyz, initial_poses_rpy, fixed));
  plant_->set_name("plant_");
  auto pr2_instance_id = world_info[0].instance_id;
  const int num_actuators = plant_->get_rigid_body_tree().get_num_actuators();
  
  // Set up connections so that the robot's state can be published and the robot can recieve plans.
  auto robot_state_pub = diagram_builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_robot_state>("ROBOT_STATE",
                                                               &lcm));
  robot_state_pub->set_name("robot_state_publisher");

  auto robot_state_sender =
      diagram_builder.AddSystem<RobotStateSender>(num_actuators);
  robot_state_sender->set_name("robot_state_sender");

  auto plan_receiver = diagram_builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<robotlocomotion::robot_plan_t>(
          "ROBOT_PLAN", &lcm));
  plan_receiver->set_name("plan_receiver");

  auto command_injector = diagram_builder.AddSystem<RobotPlanInterpolator>(
      drake::FindResourceOrThrow("drake/examples/pr2/pr2_description/urdf/pr2_with_joints_for_base_movement_and_limited_gripper_movement_no_collisions_except_grippers.urdf"));
  command_injector->set_name("command_injector");
  
  auto plan_status_pub = diagram_builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::plan_status_t>("PLAN_STATUS",
                                                               &lcm));
  plan_status_pub->set_name("plan_status_publisher");

  auto plan_status_sender = diagram_builder.AddSystem<PlanStatusSender>();
  plan_status_sender->set_name("plan_status_sender");

  diagram_builder.Connect(command_injector->get_status_output_port(), plan_status_sender->get_input_port(0));
  diagram_builder.Connect(plan_status_sender->get_output_port(0), plan_status_pub->get_input_port(0));

  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      robot_state_sender->get_state_input_port());
  diagram_builder.Connect(command_injector->get_state_output_port(),
                           robot_state_sender->get_command_input_port());
  diagram_builder.Connect(robot_state_sender->get_output_port(0),
                           robot_state_pub->get_input_port(0));
 
  // Add the controller and connect the appropriate systems together for the controller.
  VectorX<double> kp(num_actuators);
  kp << 1000, 1000, 2500, 400000, 500, 500, 2000, 2050, 1000, 1000, 200, 200,
      50, 25, 25, 2000, 2050, 1000, 1000, 200, 200, 50, 25, 25;
  VectorX<double> ki(num_actuators);
  ki << 0, 0, 5, 20000, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0, 0, 5, 5, 5,
      5, 5, 5, 5, 0, 0;
  VectorX<double> kd(num_actuators);
  kd << 1820, 1820, 750, 1, 1, 1, 7, 5, 1, 1, .2, 1, .1, 0, 0, 7, 5, 1, 1,
      .2, 1, .1, 0, 0;
  auto Binv = plant_->get_rigid_body_tree()
                  .B.block(0, 0, num_actuators, num_actuators)
                  .inverse();
  auto controller = diagram_builder.AddSystem<systems::controllers::PidController<double>>(
      std::make_unique<systems::controllers::PidController<double>>(
          Binv, MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki,
          kd));

  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      command_injector->get_state_input_port());
  diagram_builder.Connect(plan_receiver->get_output_port(0),
                           command_injector->get_plan_input_port());
  diagram_builder.Connect(command_injector->get_state_output_port(),
                           controller->get_input_port_desired_state());
  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      controller->get_input_port_estimated_state());
  diagram_builder.Connect(
      controller->get_output_port_control(),
      plant_->model_instance_actuator_command_input_port(pr2_instance_id));

  // Add visualizer.
  systems::DrakeVisualizer& visualizer_publisher = *diagram_builder.template AddSystem<systems::DrakeVisualizer>(plant_->get_rigid_body_tree(), &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  diagram_builder.Connect(plant_->state_output_port(),
                    visualizer_publisher.get_input_port(0));
 
  // Publish contact results.
  auto contact_viz =
      diagram_builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant_->get_rigid_body_tree());
  auto contact_results_publisher = diagram_builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));

  diagram_builder.Connect(plant_->contact_results_output_port(),
                           contact_viz->get_input_port(0));
  diagram_builder.Connect(contact_viz->get_output_port(0),
                           contact_results_publisher->get_input_port(0));

  // Specify contact parameters, chosen to provide a (usually) stable and fast simulation.
  const double kStaticFriction = 1.0;
  const double kDynamicFriction = 0.5;
  const double kStictionSlipTolerance = 0.001;
  plant_->set_friction_contact_parameters(kStaticFriction, kDynamicFriction, kStictionSlipTolerance);

  const double kStiffness = 1000;
  const double kDissipation = 100;
  plant_->set_normal_contact_parameters(kStiffness, kDissipation);
  
  // Create the simulation object.
  std::unique_ptr<systems::Diagram<double>> diagram = diagram_builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto context = simulator.get_mutable_context();
  
  // Specify an integrator which, along with it's settings, has been chosen because it provides a (usually) stable and fast simulation, even when gripping.
  simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      *diagram, 1e-4, context);
  
  // Set the initial joint positions.
  for (int index = 0; index < num_actuators;
       index++) {
    plant_->set_position(simulator.get_mutable_context(), index,
                        initial_robot_joint_positions[index]);
  }

  // Initialize the robot plan interpolator.
  auto& plan_source_context = diagram->GetMutableSubsystemContext(
      *command_injector, simulator.get_mutable_context());
  command_injector->Initialize(plan_source_context.get_time(),
                               initial_robot_joint_positions,
                               plan_source_context.get_mutable_state());

  // Start the simulation.
  lcm.StartReceiveThread();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(999999999999);
}
}  // namespace pr2
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  drake::examples::pr2::main(argc, argv);
  return 0;
}

/// @file
///
/// Implements a simulation of two KUKA iiwa arms (with no-end effectors)
/// and a box. Like the driver for the physical arms, this simulation
/// communicates over LCM using lcmt_iiwa_status and lcmt_iiwa_command messages
/// for the arms. It is intended to be a be a direct replacement for the
/// KUKA iiwa driver and the actual robot hardware.
///

#include <memory>

#include <gflags/gflags.h>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/dev/box_rotation/iiwa_box_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/frame_pose_tracker.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/sensors/optitrack_encoder.h"
#include "drake/systems/sensors/optitrack_sender.h"
#include "drake/util/drakeGeometryUtil.h"

DEFINE_string(urdf, "", "Name of urdf file to load");
DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate (s)");
DEFINE_double(youngs_modulus, 3e7, "Default material's Young's modulus (Pa)");
DEFINE_double(dissipation, 5, "Contact Dissipation (s/m)");
DEFINE_double(static_friction, 0.5, "Static Friction");
DEFINE_double(dynamic_friction, 0.2, "Dynamic Friction");
DEFINE_double(v_stiction_tol, 0.01, "v Stiction Tol (m/s)");
DEFINE_double(contact_area, 2e-4,
              "The characteristic scale of contact area (m^2)");
DEFINE_bool(use_visualizer, true, "Use Drake Visualizer?");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace box_rotation {
namespace {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::RigidBodyPlant;
using systems::Simulator;
using manipulation::util::FramePoseTracker;
using systems::sensors::OptitrackEncoder;
using systems::sensors::OptitrackLCMFrameSender;

const char *const kIiwaUrdf = "drake/examples/kuka_iiwa_arm/dev/box_rotation/"
    "models/dual_iiwa14_primitive_sphere_visual_collision.urdf";

template<typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(
    manipulation::util::ModelInstanceInfo<T> *iiwa_instance,
    manipulation::util::ModelInstanceInfo<T> *box_instance) {

  const std::string iiwa_path =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : kIiwaUrdf);

  auto tree_builder =
      std::make_unique<manipulation::util::WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", iiwa_path);
  tree_builder->StoreModel("table",
                           "drake/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "large_table", "drake/examples/kuka_iiwa_arm/dev/box_rotation/models/"
      "large_extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel("box",
                           "drake/examples/kuka_iiwa_arm/dev/box_rotation/"""
                           "models/box.urdf");

  // Build a world with three fixed tables.  A box is placed one on
  // table, and the iiwa arms are fixed to the other two tables.
  tree_builder->AddFixedModelInstance(
      "table", /* right arm */
      Eigen::Vector3d(0, 0, 0) /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance(
      "table", /* left arm */
      Eigen::Vector3d(0, 0.9, 0) /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance(
      "large_table", /* box */
      Eigen::Vector3d(0.72, 0.9/2, 0) /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);

  tree_builder->AddGround();

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.057m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.057 / 2.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;

  // Coordinates for the right robot arm, which is centered at x=0, y=0
  // The left arm is 0.9m away from the right arm in the y-axis (specified
  // in the urdf).
  // TODO(rcory): Could I grab the arm distance from the URDF directly?
  const Eigen::Vector3d kRobotBase(0, 0, kTableTopZInWorld);
  // Start the box on the table and place it between the two arms.
  // The distance between the two arms is y=0.9m. Each edge of the box measures
  // 0.565 m.
  const Eigen::Vector3d kBoxBase(0.7, 0.9/2 , kTableTopZInWorld + 0.565/2);

  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddFloatingModelInstance("box", kBoxBase,
                                              Vector3<double>(0, 0, 0));
  *box_instance = tree_builder->get_model_info_for_instance(id);

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());

  return plant;
}

int DoMain() {
  systems::DiagramBuilder<double> builder;

  manipulation::util::ModelInstanceInfo<double> iiwa_instance, box_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &box_instance);
  model_ptr->set_name("plant");

  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_static_friction, FLAGS_dynamic_friction);
  model_ptr->set_default_compliant_material(default_material);
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_area = FLAGS_contact_area;
  model_parameters.v_stiction_tolerance = FLAGS_v_stiction_tol;
  model_ptr->set_contact_model_parameters(model_parameters);

  auto model =
      builder.template AddSystem<IiwaAndBoxPlantWithStateEstimator<double>>(
          std::move(model_ptr), iiwa_instance, box_instance);
  model->set_name("plant_with_state_estimator");

  const RigidBodyTree<double>& tree = model->get_plant().get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;

  if (FLAGS_use_visualizer) {
    DrakeVisualizer*
        visualizer = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
    visualizer->set_name("visualizer");
    builder.Connect(model->get_output_port_plant_state(),
                    visualizer->get_input_port(0));
    visualizer->set_publish_period(kIiwaLcmStatusPeriod);
  }

  // Create the command subscriber and status publisher.
  auto iiwa_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND",
                                                                 &lcm));
  iiwa_command_sub->set_name("iiwa_command_subscriber");
  auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>(14);
  iiwa_command_receiver->set_name("iwwa_command_receiver");

  auto iiwa_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS",
                                                               &lcm));
  iiwa_status_pub->set_name("iiwa_status_publisher");
  iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>(14);
  iiwa_status_sender->set_name("iiwa_status_sender");

  // Create the Optitrack sender and publisher. The sender is configured to
  // send three objects: left arm base, right arm base, and box.
  auto optitrack_sender = builder.AddSystem<OptitrackLCMFrameSender>(3);
  optitrack_sender->set_name("optitrack frame sender");
  auto optitrack_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));
  optitrack_pub->set_name("optitrack frame publisher");
  optitrack_pub->set_publish_period(systems::sensors::kLcmStatusPeriod);

  // Create the FramePoseTracker system.
  std::map<std::string, std::pair<std::string, int>> frame_info;
  frame_info["left_iiwa_base"] = std::make_pair("left_iiwa_link_0", -1);
  frame_info["right_iiwa_base"] = std::make_pair("right_iiwa_link_0", -1);
  frame_info["box"] = std::make_pair("box", -1);
  auto pose_tracker = builder.AddSystem<FramePoseTracker>(tree, frame_info);
  pose_tracker->set_name("frame pose tracker");

  // Create the OptitrackEncoder system. This assigns a unique Optitrack ID to
  // each tracked frame (similar to the Motive software). These are used to
  // create a tracked Optitrack body.
  std::map<std::string, int> frame_name_to_id_map;
  frame_name_to_id_map["left_iiwa_base"] = 1;
  frame_name_to_id_map["right_iiwa_base"] = 2;
  frame_name_to_id_map["box"] = 3;
  auto optitrack_encoder =
      builder.AddSystem<OptitrackEncoder>(frame_name_to_id_map);
  optitrack_encoder->set_name("optitrack encoder");

  // Connect the systems related to tracking bodies.
  builder.Connect(model->get_output_port_kinematics_results(),
                  pose_tracker->get_kinematics_input_port());
  builder.Connect(pose_tracker->get_pose_bundle_output_port(),
                  optitrack_encoder->get_pose_bundle_input_port());
  builder.Connect(optitrack_encoder->get_optitrack_output_port(),
                  optitrack_sender->get_optitrack_input_port());
  builder.Connect(optitrack_sender->get_lcm_output_port(),
                  optitrack_pub->get_input_port(0));

  // TODO(rcory): Do we need this accel source?
  auto iiwa_zero_acceleration_source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(14));
  iiwa_zero_acceleration_source->set_name("zero_acceleration");

  builder.Connect(iiwa_command_sub->get_output_port(0),
                  iiwa_command_receiver->get_input_port(0));

  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  model->get_input_port_iiwa_state_command());

  builder.Connect(iiwa_zero_acceleration_source->get_output_port(),
                  model->get_input_port_iiwa_acceleration_command());

  builder.Connect(model->get_output_port_iiwa_state(),
                  iiwa_status_sender->get_state_input_port());

  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  iiwa_status_sender->get_command_input_port());

  builder.Connect(iiwa_status_sender->get_output_port(0),
                  iiwa_status_pub->get_input_port(0));

  auto iiwa_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "IIWA_STATE_EST", &lcm));
  iiwa_state_pub->set_name("iiwa_state_publisher");
  iiwa_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(model->get_output_port_iiwa_robot_state_msg(),
                  iiwa_state_pub->get_input_port(0));
  iiwa_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

  auto box_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "OBJECT_STATE_EST", &lcm));
  box_state_pub->set_name("box_state_publisher");
  box_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(model->get_output_port_box_robot_state_msg(),
                  box_state_pub->get_input_port(0));
  box_state_pub->set_publish_period(kIiwaLcmStatusPeriod);


  // Add contact viz.
  if (FLAGS_use_visualizer) {
    auto contact_viz =
        builder.template AddSystem<systems::ContactResultsToLcmSystem<double>>(
            model->get_tree());
    contact_viz->set_name("contact_viz");

    auto contact_results_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", &lcm));
    contact_results_publisher->set_name("contact_results_publisher");

    builder.Connect(model->get_output_port_contact_results(),
                    contact_viz->get_input_port(0));
    builder.Connect(contact_viz->get_output_port(0),
                    contact_results_publisher->get_input_port(0));
    contact_results_publisher->set_publish_period(.01);
  }

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
      *sys, 1e-3, &simulator.get_mutable_context());

  // TODO(rcory): Explore other integration schemes here.
//  simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
//      *sys, simulator.get_mutable_context());
//  simulator.get_mutable_integrator()->set_target_accuracy(1e-2);
//  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-3);

  lcm.StartReceiveThread();
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace box_rotation
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::box_rotation::DoMain();
}

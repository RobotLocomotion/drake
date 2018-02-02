/// @file
///
/// Implements a simulation of the KUKA iiwa arm with a Schunk WSG 50
/// attached as an end effector.  Like the driver for the physical
/// arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages for the arm, and the
/// lcmt_schunk_status and lcmt_schunk_command messages for the
/// gripper. It is intended to be a be a direct replacement for the
/// KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/util/drakeGeometryUtil.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(dt, 1e-3, "Integration step size");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using manipulation::schunk_wsg::SchunkWsgStatusSender;
using manipulation::schunk_wsg::SchunkWsgController;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::ModelInstanceInfo;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::RigidBodyPlant;
using systems::RungeKutta2Integrator;
using systems::Simulator;

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

// TODO(naveen): refactor this to reduce duplicate code.
template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(
    ModelInstanceInfo<T>* iiwa_instance, ModelInstanceInfo<T>* wsg_instance,
    ModelInstanceInfo<T>* box_instance) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);
  tree_builder->StoreDrakeModel(
      "table",
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreDrakeModel(
      "box",
      "drake/examples/kuka_iiwa_arm/models/objects/"
      "block_for_pick_and_place.urdf");
  tree_builder->StoreDrakeModel(
      "wsg",
      "drake/manipulation/models/wsg_50_description"
      "/sdf/schunk_wsg_50_ball_contact.sdf");

  // Build a world with two fixed tables.  A box is placed one on
  // table, and the iiwa arm is fixed to the other.
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d::Zero() /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d(0.8, 0, 0) /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d(0, 0.85, 0) /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);

  tree_builder->AddGround();

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.057m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.057 / 2.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;

  // Coordinates for kRobotBase originally from iiwa_world_demo.cc.
  // The intention is to center the robot on the table.
  const Eigen::Vector3d kRobotBase(0, 0, kTableTopZInWorld);
  // Start the box slightly above the table.  If we place it at
  // the table top exactly, it may start colliding the table (which is
  // not good, as it will likely shoot off into space).
  const Eigen::Vector3d kBoxBase(0.8, 0., kTableTopZInWorld + 0.1);

  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddFloatingModelInstance("box", kBoxBase,
                                              Vector3<double>(0, 0, 0));
  *box_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(id);

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());

  return plant;
}

int DoMain() {
  systems::DiagramBuilder<double> builder;

  ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance, &box_instance);
  model_ptr->set_name("plant");

  auto model =
      builder.template AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
          std::move(model_ptr), iiwa_instance, wsg_instance, box_instance);
  model->set_name("plant_with_state_estimator");

  const RigidBodyTree<double>& tree = model->get_plant().get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  visualizer->set_name("visualizer");
  builder.Connect(model->get_output_port_plant_state(),
                  visualizer->get_input_port(0));
  visualizer->set_publish_period(kIiwaLcmStatusPeriod);

  // Create the command subscriber and status publisher.
  auto iiwa_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND",
                                                                 &lcm));
  iiwa_command_sub->set_name("iiwa_command_subscriber");
  auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>();
  iiwa_command_receiver->set_name("iwwa_command_receiver");

  auto iiwa_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS",
                                                               &lcm));
  iiwa_status_pub->set_name("iiwa_status_publisher");
  iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>();
  iiwa_status_sender->set_name("iiwa_status_sender");

  std::vector<int> instance_ids = {iiwa_instance.instance_id};
  auto external_torque_converter =
      builder.AddSystem<IiwaContactResultsToExternalTorque>(
          tree, instance_ids);

  // TODO(siyuan): Connect this to kuka_planner runner once it generates
  // reference acceleration.
  auto iiwa_zero_acceleration_source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(7));
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
  builder.Connect(model->get_output_port_computed_torque(),
                  iiwa_status_sender->get_commanded_torque_input_port());
  builder.Connect(model->get_output_port_iiwa_measured_torque(),
                  iiwa_status_sender->get_measured_torque_input_port());
  builder.Connect(model->get_output_port_contact_results(),
                  external_torque_converter->get_input_port(0));
  builder.Connect(external_torque_converter->get_output_port(0),
                  iiwa_status_sender->get_external_torque_input_port());
  builder.Connect(iiwa_status_sender->get_output_port(0),
                  iiwa_status_pub->get_input_port(0));

  auto wsg_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", &lcm));
  wsg_command_sub->set_name("wsg_command_subscriber");
  auto wsg_controller = builder.AddSystem<SchunkWsgController>();

  auto wsg_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", &lcm));
  wsg_status_pub->set_name("wsg_status_publisher");
  wsg_status_pub->set_publish_period(
      manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod);

  auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
      model->get_output_port_wsg_state().size(),
      manipulation::schunk_wsg::kSchunkWsgPositionIndex,
      manipulation::schunk_wsg::kSchunkWsgVelocityIndex);
  wsg_status_sender->set_name("wsg_status_sender");

  builder.Connect(wsg_command_sub->get_output_port(0),
                  wsg_controller->get_command_input_port());
  builder.Connect(wsg_controller->get_output_port(0),
                  model->get_input_port_wsg_command());
  builder.Connect(model->get_output_port_wsg_state(),
                  wsg_status_sender->get_input_port(0));
  builder.Connect(model->get_output_port_wsg_measured_torque(),
                  wsg_status_sender->get_input_port(1));
  builder.Connect(model->get_output_port_wsg_state(),
                  wsg_controller->get_state_input_port());
  builder.Connect(*wsg_status_sender, *wsg_status_pub);

  auto iiwa_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));
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

  builder.Connect(model->get_output_port_object_robot_state_msg(),
                  box_state_pub->get_input_port(0));
  box_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();
  // When using the default RK3 integrator, the simulation stops
  // advancing once the gripper grasps the box.  Grasping makes the
  // problem computationally stiff, which brings the default RK3
  // integrator to its knees.
  simulator.reset_integrator<RungeKutta2Integrator<double>>(*sys,
      FLAGS_dt, &simulator.get_mutable_context());
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::DoMain();
}

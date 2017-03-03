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

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

#include "drake/util/drakeGeometryUtil.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using schunk_wsg::SchunkWsgStatusSender;
using schunk_wsg::SchunkWsgTrajectoryGenerator;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::InputPortDescriptor;
using systems::OutputPortDescriptor;
using systems::RigidBodyPlant;
using systems::Simulator;

const char* const kIiwaUrdf =
    "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf";

// TODO(naveen): refactor this to reduce duplicate code.
template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(
    ModelInstanceInfo<T>* iiwa_instance, ModelInstanceInfo<T>* wsg_instance,
    ModelInstanceInfo<T>* box_instance) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "box",
      "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf");
  tree_builder->StoreModel(
      "wsg", "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

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
  const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);
  // Start the box slightly above the table.  If we place it at
  // the table top exactly, it may start colliding the table (which is
  // not good, as it will likely shoot off into space).
  const Eigen::Vector3d kBoxBase(1 + -0.43, -0.65, kTableTopZInWorld + 0.1);

  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddFloatingModelInstance("box", kBoxBase,
                                              Vector3<double>(0, 0, 1));
  *box_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddModelInstanceToFrame(
      "wsg", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(id);

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());

  return plant;
}

// TODO(sam.creasey) We should de-duplicate this with kuka_demo.cc.
// See #4521 which moves the factory for the KukaDemo into a header,
// we might want to examine/duplicate that approach.
template <typename T>
class SimulatedIiwaWithWsg : public systems::Diagram<T> {
 public:
  SimulatedIiwaWithWsg() {
    this->set_name("SimulatedIiwaWithWsg");
    DiagramBuilder<T> builder;

    ModelInstanceInfo<T> iiwa_info, wsg_info, box_info;
    plant_ = builder.AddSystem(
        BuildCombinedPlant<T>(&iiwa_info, &wsg_info, &box_info));
    const auto& iiwa_input_port =
        plant_->model_instance_actuator_command_input_port(
            iiwa_info.instance_id);
    const auto& iiwa_output_port =
        plant_->model_instance_state_output_port(iiwa_info.instance_id);

    const auto& wsg_input_port =
        plant_->model_instance_actuator_command_input_port(
            wsg_info.instance_id);
    const auto& wsg_output_port =
        plant_->model_instance_state_output_port(wsg_info.instance_id);

    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
    // Uses integral gains to deal with the added mass from the grasped object.
    iiwa_ki << 1, 1, 1, 1, 1, 1, 1;

    // Exposing feedforward acceleration. Should help with more dynamic
    // motions.
    auto iiwa_controller =
        builder.template AddSystem<systems::InverseDynamicsController<T>>(
            iiwa_info.model_path, iiwa_info.world_offset, iiwa_kp, iiwa_ki,
            iiwa_kd, true /* with feedforward acceleration */);

    // Sets a zero configuration and computes spatial inertia for the gripper
    // as well as the pose of the end effector link of iiwa using the world
    // tree.
    const RigidBodyTree<T>& world_tree = plant_->get_rigid_body_tree();
    KinematicsCache<T> world_cache = world_tree.CreateKinematicsCache();
    world_cache.initialize(world_tree.getZeroConfiguration());
    world_tree.doKinematics(world_cache);

    const RigidBody<T>* end_effector = world_tree.FindBody("iiwa_link_7");
    Isometry3<T> X_WEE =
        world_tree.CalcBodyPoseInWorldFrame(world_cache, *end_effector);

    // The inertia of the added gripper is lumped into the last link of the
    // controller's iiwa arm model. This is motivated by the fact that the
    // gripper inertia is relatively large compared to the last couple links
    // in the iiwa arm model. And to completely rely on using feedback to cope
    // with added inertia, we need to either rely on larger gains (which will
    // cause simulation to explode without the gripper), or wait longer for
    // the integrator to kick in.

    // Computes the lumped inertia for the gripper.
    std::set<int> gripper_instance_set = {wsg_info.instance_id};
    Matrix6<T> lumped_gripper_inertia_W =
        world_tree.LumpedSpatialInertiaInWorldFrame(
            world_cache, gripper_instance_set);
    // Transfer it to the last iiwa link's body frame.
    Matrix6<T> lumped_gripper_inertia_EE =
        transformSpatialInertia(X_WEE.inverse(), lumped_gripper_inertia_W);
    lumped_gripper_inertia_EE += end_effector->get_spatial_inertia();

    // Changes the controller's iiwa end effector's link to the lumped inertia.
    RigidBody<T>* controller_ee =
        iiwa_controller->get_robot_for_control().FindBody("iiwa_link_7");
    controller_ee->set_spatial_inertia(lumped_gripper_inertia_EE);

    // Connect iiwa controller and robot.
    builder.Connect(iiwa_output_port,
                    iiwa_controller->get_input_port_estimated_state());
    builder.Connect(iiwa_controller->get_output_port_control(),
                    iiwa_input_port);

    // Export iiwa's desired state input, and state output.
    builder.ExportInput(iiwa_controller->get_input_port_desired_state());
    builder.ExportInput(iiwa_controller->get_input_port_desired_acceleration());
    builder.ExportOutput(iiwa_output_port);

    // Sets up the WSG gripper part.
    const std::map<std::string, int> index_map =
        world_tree.computePositionNameToIndexMap();
    const int left_finger_position_index =
        index_map.at("left_finger_sliding_joint");
    const int position_index = plant_->FindInstancePositionIndexFromWorldIndex(
        wsg_info.instance_id, left_finger_position_index);
    const int velocity_index =
        position_index + plant_->get_num_positions(wsg_info.instance_id);

    Eigen::MatrixXd feedback_matrix = Eigen::MatrixXd::Zero(
        2 * plant_->get_num_actuators(wsg_info.instance_id),
        2 * plant_->get_num_positions(wsg_info.instance_id));
    feedback_matrix(0, position_index) = 1.;
    feedback_matrix(1, velocity_index) = 1.;
    std::unique_ptr<systems::MatrixGain<T>> feedback_selector =
        std::make_unique<systems::MatrixGain<T>>(feedback_matrix);

    // TODO(sam.creasey) The choice of constants below is completely
    // arbitrary and may not match the performance of the actual
    // gripper.
    const T wsg_kp = 300.0;   // This seems very high, for some grasps
                              // it's actually in the right power of
                              // two.  We'll need to revisit this once
                              // we're using the force command sent to
                              // the gripper properly.
    const T wsg_ki = 0.0;
    const T wsg_kd = 5.0;
    const VectorX<T> wsg_v = VectorX<T>::Ones(wsg_input_port.size());

    auto wsg_controller = builder.template AddSystem<systems::PidController<T>>(
        std::move(feedback_selector), wsg_v * wsg_kp, wsg_v * wsg_ki,
        wsg_v * wsg_kd);

    // Connects WSG and controller.
    builder.Connect(wsg_output_port,
                    wsg_controller->get_input_port_estimated_state());
    builder.Connect(wsg_controller->get_output_port_control(), wsg_input_port);

    //  Export wsg's desired state input, and state output.
    builder.ExportInput(wsg_controller->get_input_port_desired_state());
    builder.ExportOutput(wsg_output_port);

    builder.ExportOutput(plant_->get_output_port(0));

    // Sets up a "state estimator" for iiwa that generates
    // bot_core::robot_state_t messages.
    auto iiwa_state_est =
        builder.template AddSystem<OracularStateEstimation<T>>(
            iiwa_controller->get_robot_for_control(),
            iiwa_controller->get_robot_for_control().get_body(1));
    builder.Connect(iiwa_output_port, iiwa_state_est->get_input_port_state());
    builder.ExportOutput(iiwa_state_est->get_output_port_msg());

    // Sets up a "state estimator" for the box that generates
    // bot_core::robot_state_t messages.
    // Make a box RBT for the fake state estimator.
    object_ = std::make_unique<RigidBodyTree<T>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        box_info.model_path, multibody::joints::kQuaternion,
        box_info.world_offset, object_.get());
    auto box_state_est = builder.template AddSystem<OracularStateEstimation<T>>(
        *object_, object_->get_body(1));
    builder.Connect(
        plant_->model_instance_state_output_port(box_info.instance_id),
        box_state_est->get_input_port_state());
    builder.ExportOutput(box_state_est->get_output_port_msg());

    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_plant() const { return *plant_; }

  const InputPortDescriptor<T>& get_iiwa_state_input_port() const {
    return this->get_input_port(0);
  }

  const InputPortDescriptor<T>& get_iiwa_acceleration_input_port() const {
    return this->get_input_port(1);
  }

  const OutputPortDescriptor<T>& get_iiwa_state_port() const {
    return this->get_output_port(0);
  }

  const InputPortDescriptor<T>& get_wsg_input_port() const {
    return this->get_input_port(2);
  }

  const OutputPortDescriptor<T>& get_wsg_state_port() const {
    return this->get_output_port(1);
  }

  const OutputPortDescriptor<T>& get_plant_output_port() const {
    return this->get_output_port(2);
  }

  const OutputPortDescriptor<T>& get_iiwa_robot_state_msg_port() const {
    return this->get_output_port(3);
  }

  const OutputPortDescriptor<T>& get_box_robot_state_msg_port() const {
    return this->get_output_port(4);
  }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  std::unique_ptr<RigidBodyTree<T>> object_;
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto model = builder.AddSystem<SimulatedIiwaWithWsg<double>>();

  const RigidBodyTree<double>& tree = model->get_plant().get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  builder.Connect(model->get_plant_output_port(),
                  visualizer->get_input_port(0));

  // Create the command subscriber and status publisher.
  auto iiwa_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>(
          "IIWA_COMMAND", &lcm));
  auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>();

  auto iiwa_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
          "IIWA_STATUS", &lcm));
  iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>();

  // TODO(siyuan): Connect this to kuka_planner runner once it generates
  // reference acceleration.
  auto iiwa_zero_acceleration_source =
        builder.template AddSystem<systems::ConstantVectorSource<double>>(
            Eigen::VectorXd::Zero(7));

  builder.Connect(iiwa_command_sub->get_output_port(0),
                  iiwa_command_receiver->get_input_port(0));
  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  model->get_iiwa_state_input_port());
  builder.Connect(iiwa_zero_acceleration_source->get_output_port(),
                  model->get_iiwa_acceleration_input_port());

  builder.Connect(model->get_iiwa_state_port(),
                  iiwa_status_sender->get_state_input_port());
  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  iiwa_status_sender->get_command_input_port());
  builder.Connect(iiwa_status_sender->get_output_port(0),
                  iiwa_status_pub->get_input_port(0));

  auto wsg_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", &lcm));
  auto wsg_trajectory_generator =
      builder.AddSystem<SchunkWsgTrajectoryGenerator>(
          model->get_wsg_state_port().size(), 0);

  auto wsg_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", &lcm));
  auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
      model->get_wsg_state_port().size(), 0, 1);

  builder.Connect(wsg_command_sub->get_output_port(0),
                  wsg_trajectory_generator->get_command_input_port());
  builder.Connect(wsg_trajectory_generator->get_output_port(0),
                  model->get_wsg_input_port());
  builder.Connect(model->get_wsg_state_port(),
                  wsg_status_sender->get_input_port(0));
  builder.Connect(model->get_wsg_state_port(),
                  wsg_trajectory_generator->get_state_input_port());
  builder.Connect(*wsg_status_sender, *wsg_status_pub);

  auto iiwa_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "IIWA_STATE_EST", &lcm));
  builder.Connect(model->get_iiwa_robot_state_msg_port(),
                  iiwa_state_pub->get_input_port(0));

  auto box_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "OBJECT_STATE_EST", &lcm));
  builder.Connect(model->get_box_robot_state_msg_port(),
                  box_state_pub->get_input_port(0));

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}

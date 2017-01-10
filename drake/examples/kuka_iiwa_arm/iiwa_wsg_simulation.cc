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
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using schunk_wsg::SchunkWsgStatusSender;
using schunk_wsg::SchunkWsgTrajectoryGenerator;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::GravityCompensator;
using systems::InputPortDescriptor;
using systems::MatrixGain;
using systems::Multiplexer;
using systems::OutputPortDescriptor;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::Simulator;

const char* const kIiwaUrdf =
    "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";

template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(
    int* iiwa_instance_id, int* wsg_instance_id) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel(
      "table",
      "/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");
  tree_builder->StoreModel(
      "wsg", "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

  // Build a world with two fixed tables.  A cylinder is placed one on
  // table, and the iiwa arm is fixed to the other.
  tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d::Zero() /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d(1, 0, 0) /* xyz */,
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
  // Start the cylinder slightly above the table.  If we place it at
  // the table top exactly, it may start colliding the table (which is
  // not good, as it will likely shoot off into space).
  const Eigen::Vector3d kCylinderBase(
      1 + -0.43, -0.65, kTableTopZInWorld + 0.1);

  *iiwa_instance_id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  tree_builder->AddFloatingModelInstance("cylinder", kCylinderBase);
  *wsg_instance_id = tree_builder->AddModelInstanceToFrame(
      "wsg", Eigen::Vector3d::Zero(),  Eigen::Vector3d::Zero(),
      tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());
  // Arbitrary contact parameters.
  plant->set_contact_parameters(10000., 100., 10.);
  return plant;
}

// TODO(sam.creasey) We should de-duplicate this with kuka_demo.cc.
// See #4521 which moves the factory for the KukaDemo into a header,
// we might want to examine/duplicate that approach.
template<typename T>
class SimulatedIiwaWithWsg : public systems::Diagram<T> {
 public:
  SimulatedIiwaWithWsg() {
    this->set_name("SimulatedIiwaWithWsg");
    DiagramBuilder<T> builder;

    int iiwa_instance_id{};
    int wsg_instance_id{};
    plant_ = builder.AddSystem(BuildCombinedPlant<T>(&iiwa_instance_id,
                                                     &wsg_instance_id));
    const auto& iiwa_input_port = plant_->model_input_port(iiwa_instance_id);
    const auto& iiwa_output_port =
        plant_->model_state_output_port(iiwa_instance_id);

    const auto& wsg_input_port = plant_->model_input_port(wsg_instance_id);
    const auto& wsg_output_port =
            plant_->model_state_output_port(wsg_instance_id);

    // Connect the pid controllers for each device.

    // Constants are chosen by trial and error to qualitatively match
    // an experimental run with the same initial conditions and
    // planner.  It's still not a very good match.  Quantitative
    // comparisons would require torque control and a more careful
    // estimation of the model parameters such as friction in the
    // joints.
    Eigen::VectorXd iiwa_kp = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd iiwa_kd = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd iiwa_ki = Eigen::VectorXd::Zero(7);
    iiwa_kp << 100, 200, 100, 200, 10, 100, 1;
    iiwa_ki << 0, 2, 0, 1, 0, 0.5, 0;
    for (int i = 0; i < iiwa_kp.size(); i++) {
      iiwa_kd[i] = std::sqrt(iiwa_kp[i]);
    }

    auto iiwa_pid_ports = PidControlledSystem<T>::ConnectController(
        iiwa_input_port, iiwa_output_port, nullptr /* feedback */,
        iiwa_kp, iiwa_ki, iiwa_kd, &builder);

    const RigidBodyTree<T>& tree = plant_->get_rigid_body_tree();
    const std::map<std::string, int> index_map =
        tree.computePositionNameToIndexMap();
    const int left_finger_position_index =
        index_map.at("left_finger_sliding_joint");
    const int position_index = plant_->FindInstancePositionIndexFromWorldIndex(
        wsg_instance_id, left_finger_position_index);
    const int velocity_index = position_index +
        plant_->get_num_positions(wsg_instance_id);

    Eigen::MatrixXd feedback_matrix = Eigen::MatrixXd::Zero(
        2 * plant_->get_num_actuators(wsg_instance_id),
        2 * plant_->get_num_positions(wsg_instance_id));
    feedback_matrix(0, position_index) = 1.;
    feedback_matrix(1, velocity_index) = 1.;
    std::unique_ptr<MatrixGain<T>> feedback_selector =
        std::make_unique<MatrixGain<T>>(feedback_matrix);

    // TODO(sam.creasey) The choice of constants below is completely
    // arbitrary and may not match the performance of the actual
    // gripper.
    const T wsg_kp = 3000.0;  // This seems very high, for some grasps
                              // it's actually in the right power of
                              // two.  We'll need to revisit this once
                              // we're using the force command sent to
                              // the gripper properly.
    const T wsg_ki = 0.0;
    const T wsg_kd = 5.0;
    const VectorX<T> wsg_v = VectorX<T>::Ones(wsg_input_port.size());
    auto wsg_pid_ports = PidControlledSystem<T>::ConnectController(
        wsg_input_port, wsg_output_port, std::move(feedback_selector),
        wsg_v * wsg_kp, wsg_v * wsg_ki, wsg_v * wsg_kd,
        &builder);

    // Create a tree containing only the iiwa to use with the gravity
    // compensator.
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + kIiwaUrdf,
        drake::multibody::joints::kFixed,
        nullptr /* weld to frame */, &iiwa_tree_);

    // Create a multiplexer to handle the fact that we'll be getting
    // the input state for the positions and velocities from different
    // sources.  Port 0 (positions) will be exported as an input to
    // the diagram.  Port 1 (velocities) is connected below).
    auto input_mux = builder.template AddSystem<Multiplexer<T>>(
        std::vector<int>{iiwa_tree_.get_num_positions(),
              iiwa_tree_.get_num_velocities()});
    builder.Connect(input_mux->get_output_port(0),
                    iiwa_pid_ports.state_input_port);

    // The iiwa's control protocol doesn't have any way to express the
    // desired velocity for the arm, so this simulation doesn't take
    // target velocities as an input.  The PidControlledSystem does
    // want target velocities to calculate the D term.  Since we don't
    // have any logic to calculate the desired target velocity (yet!)
    // set the D term (to stabilize the arm near the commanded
    // position) and feed a desired velocity vector of zero.
    auto zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
        Eigen::VectorXd::Zero(iiwa_tree_.get_num_velocities()));
    builder.Connect(zero_source->get_output_port(),
                    input_mux->get_input_port(1));

    auto gravity_compensator =
        builder.template AddSystem<GravityCompensator<T>>(iiwa_tree_);

    // Split the input state into two signals one with the positions and one
    // with the velocities.

    // TODO(sam.creasey) Is this a common enough thing to need to do
    // that the plant might want to output the positions and
    // velocities on separate ports?
    auto rbp_state_demux = builder.template AddSystem<Demultiplexer<T>>(
        iiwa_tree_.get_num_positions() + iiwa_tree_.get_num_velocities(),
        iiwa_tree_.get_num_positions());
    builder.Connect(iiwa_output_port, rbp_state_demux->get_input_port(0));

    // Connects the gravity compensator to the output generalized positions.
    builder.Connect(rbp_state_demux->get_output_port(0),
                    gravity_compensator->get_input_port(0));
    builder.Connect(gravity_compensator->get_output_port(0),
                    iiwa_pid_ports.control_input_port);

    builder.ExportInput(input_mux->get_input_port(0));
    builder.ExportOutput(iiwa_output_port);

    // Now finish building the WSG's part of the diagram.

    // Create a source to emit a single zero.  We'll need this to
    // express external commanded force to the PidControlledSystem.
    auto wsg_zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
        Eigen::VectorXd::Zero(1));
    builder.Connect(wsg_zero_source->get_output_port(),
                    wsg_pid_ports.control_input_port);
    builder.ExportInput(wsg_pid_ports.state_input_port);
    builder.ExportOutput(wsg_output_port);

    builder.ExportOutput(plant_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_plant() const { return *plant_; }

  const InputPortDescriptor<T>& get_iiwa_input_port() const {
    return this->get_input_port(0);
  }

  const OutputPortDescriptor<T>& get_iiwa_state_port() const {
    return this->get_output_port(0);
  }

  const InputPortDescriptor<T>& get_wsg_input_port() const {
    return this->get_input_port(1);
  }

  const OutputPortDescriptor<T>& get_wsg_state_port() const {
    return this->get_output_port(1);
  }

  const OutputPortDescriptor<T>& get_plant_output_port() const {
    return this->get_output_port(2);
  }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  PidControlledSystem<T>* controller_{nullptr};
  RigidBodyTree<T> iiwa_tree_;
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto model = builder.AddSystem<SimulatedIiwaWithWsg<double>>();

  const RigidBodyTree<double>& tree =
      model->get_plant().get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer =
      builder.AddSystem<DrakeVisualizer>(tree, &lcm);
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
  auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>();

  builder.Connect(iiwa_command_sub->get_output_port(0),
                  iiwa_command_receiver->get_input_port(0));
  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  model->get_iiwa_input_port());
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

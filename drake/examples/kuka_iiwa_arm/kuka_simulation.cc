/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;

// TODO(sam.creasey) We should de-duplicate this with kuka_demo.cc.
// I'm holding off for now because I still need to investigate how to
// create the combined Schunk+iiwa system, and I'm not sure what
// allowances I'll need in the simulation setup to get that to work.
template<typename T>
class SimulatedKuka : public systems::Diagram<T> {
 public:
  SimulatedKuka() {
    this->set_name("SimulatedKuka");
    DiagramBuilder<T> builder;

    // Instantiates a model of the world.
    auto rigid_body_tree = std::make_unique<RigidBodyTree<double>>();
    const std::string model_path = drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/iiwa14/"
        "iiwa14_simplified_collision.urdf";

    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        model_path,
        drake::multibody::joints::kFixed,
        nullptr /* weld to frame */, rigid_body_tree.get());

    drake::multibody::AddFlatTerrainToWorld(rigid_body_tree.get());

    plant_ = builder.template AddSystem<systems::RigidBodyPlant<double>>(
        std::move(rigid_body_tree));

    DRAKE_ASSERT(plant_->actuator_command_input_port().size() ==
        plant_->get_num_positions());

    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    SetPositionControlledIiwaGains(&kp, &ki, &kd);

    controller_ =
        builder.template AddSystem<systems::InverseDynamicsController<T>>(
            plant_->get_rigid_body_tree(), kp, ki, kd,
            false /* no feedforward acceleration */);

    // Connects plant and controller.
    builder.Connect(plant_->state_output_port(),
                    controller_->get_input_port_estimated_state());
    builder.Connect(controller_->get_output_port_control(),
                    plant_->actuator_command_input_port());

    // Exposes desired state input port.
    builder.ExportInput(controller_->get_input_port_desired_state());
    builder.ExportOutput(plant_->state_output_port());
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_plant() const { return *plant_; }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  systems::InverseDynamicsController<T>* controller_{nullptr};
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto model = builder.AddSystem<SimulatedKuka<double>>();

  const RigidBodyTree<double>& tree =
      model->get_plant().get_rigid_body_tree();
  VerifyIiwaTree(tree);

  // Creates and adds LCM publisher for visualization.
  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer =
      builder.AddSystem<DrakeVisualizer>(tree, &lcm);

  // Create the command subscriber and status publisher.
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>(
          "IIWA_COMMAND", &lcm));
  auto command_receiver = builder.AddSystem<IiwaCommandReceiver>();
  auto status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
          "IIWA_STATUS", &lcm));
  status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto status_sender = builder.AddSystem<IiwaStatusSender>();

  builder.Connect(command_sub->get_output_port(0),
                  command_receiver->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(0),
                  model->get_input_port(0));
  builder.Connect(model->get_output_port(0),
                  visualizer->get_input_port(0));
  builder.Connect(model->get_output_port(0),
                  status_sender->get_state_input_port());
  builder.Connect(command_receiver->get_output_port(0),
                  status_sender->get_command_input_port());
  builder.Connect(status_sender->get_output_port(0),
                  status_pub->get_input_port(0));
  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();

  command_receiver->set_initial_position(
      sys->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                      command_receiver),
      VectorX<double>::Zero(tree.get_num_positions()));


  // Simulate for a very long time.
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

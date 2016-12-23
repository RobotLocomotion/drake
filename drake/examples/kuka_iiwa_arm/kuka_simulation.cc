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
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

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
using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::GravityCompensator;
using systems::Multiplexer;
using systems::PidControlledSystem;
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

    // The following curly brace defines a scope that quarantines local
    // variables `rigid_body_tree` and `plant`, which are of type
    // `std::unique_ptr`. Ownership of `rigid_body_tree` is passed to `plant`
    // and ownership of `plant` is passed to `controller`. We quarantine these
    // local variables to prevent downstream code from seg faulting by trying to
    // access `rigid_body_tree` or `plant` after ownership is transferred.
    {
      // Instantiates a model of the world.
      auto rigid_body_tree = std::make_unique<RigidBodyTree<double>>();
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() +
          "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf",
          drake::multibody::joints::kFixed,
          nullptr /* weld to frame */, rigid_body_tree.get());

      drake::multibody::AddFlatTerrainToWorld(rigid_body_tree.get());

      // Instantiates a RigidBodyPlant from an MBD model of the world.
      auto plant = std::make_unique<RigidBodyPlant<T>>(
          std::move(rigid_body_tree));
      plant_ = plant.get();

      DRAKE_ASSERT(plant_->get_input_port(0).get_size() ==
                   plant_->get_num_positions());

      // Constants are chosen by trial and error to qualitatively match
      // an experimental run with the same initial conditions and
      // planner.  Quantitative comparisons would require torque control
      // and a more careful estimation of the model constants such as
      // friction in the joints.
      const double kp = 2.0;  // proportional constant.
      const double ki = 0.0;  // integral constant.
      const double kd = 1.0;  // derivative constant.  See below!
      controller_ = builder.template AddSystem<PidControlledSystem<T>>(
          std::move(plant), kp, ki, kd);
    }
    DRAKE_DEMAND(controller_ != nullptr);
    const RigidBodyTreed& tree = plant_->get_rigid_body_tree();

    // TODO(liang.fok): Modify controller to provide named accessors to these
    // ports.
    const int kControllerFeedforwardInputPort = 0;
    const int kControllerFeedbackInputPort = 1;

    // The iiwa's control protocol doesn't have any way to express the
    // desired velocity for the arm, so this simulation doesn't take
    // target velocities as an input.  The PidControlledSystem does
    // want target velocities to calculate the D term.  Since we don't
    // have any logic to calculate the desired target velocity (yet!)
    // set the D term (to stabilize the arm near the commanded
    // position) and feed a desired velocity vector of zero.
    auto zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
        Eigen::VectorXd::Zero(tree.get_num_velocities()));
    auto input_mux = builder.template AddSystem<Multiplexer<T>>(
        std::vector<int>{tree.get_num_positions(), tree.get_num_velocities()});
    builder.Connect(zero_source->get_output_port(),
                    input_mux->get_input_port(1));
    builder.Connect(input_mux->get_output_port(0),
                    controller_->get_input_port(kControllerFeedbackInputPort));

    auto gravity_compensator =
        builder.template AddSystem<GravityCompensator<T>>(tree);

    // Split the input state into two signals one with the positions and one
    // with the velocities.
    // For Kuka:
    // -  get_num_states() = 14
    // -  get_num_positions() = 7
    auto rbp_state_demux = builder.template AddSystem<Demultiplexer<T>>(
        plant_->get_num_states(), plant_->get_num_positions());
    builder.Connect(controller_->get_output_port(0),
                    rbp_state_demux->get_input_port(0));

    // Connects the gravity compensator to the output generalized positions.
    builder.Connect(rbp_state_demux->get_output_port(0),
                    gravity_compensator->get_input_port(0));
    builder.Connect(gravity_compensator->get_output_port(0),
                    controller_->get_input_port(
                        kControllerFeedforwardInputPort));

    builder.ExportInput(input_mux->get_input_port(0));
    builder.ExportOutput(controller_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_plant() const { return *plant_; }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  PidControlledSystem<T>* controller_{nullptr};
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

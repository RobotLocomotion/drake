/// @file
///
/// Implements a simulation of the Schunk WSG 50 gripper.  Like the
/// driver for the physical gripper, this simulation communicates over
/// LCM using the lcmt_schunk_status and lcmt_schunk_command messages.
/// It is intended to be a direct replacement for the Schunk WSG
/// driver.

#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/schunk_wsg/schunk_wsg_constants.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/examples/schunk_wsg/simulated_schunk_wsg_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::MatrixGain;
using systems::Multiplexer;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::Simulator;

template <typename T>
class PidControlledSchunkWsg : public systems::Diagram<T> {
 public:
  PidControlledSchunkWsg() {
    this->set_name("PidControlledSchunkWsg");

    DiagramBuilder<T> builder;

    std::unique_ptr<RigidBodyPlant<T>> plant =
        CreateSimulatedSchunkWsgSystem<T>();
    plant_ = plant.get();

    // Create a source to emit a single zero.  We'll need this to
    // express external commanded force to the PidControlledSystem.
    auto zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
        Eigen::VectorXd::Zero(1));

    const Eigen::MatrixXd feedback_matrix = GetSchunkWsgFeedbackSelector<T>();
    std::unique_ptr<MatrixGain<T>> feedback_selector =
        std::make_unique<MatrixGain<T>>(feedback_matrix);

    // TODO(sam.creasey) The choice of constants below is completely
    // arbitrary and may not match the performance of the actual
    // gripper.
    const T kp = 20.0;
    const T ki = 0.0;
    const T kd = 5.0;
    controller_ = builder.template AddSystem<PidControlledSystem<T>>(
        std::move(plant), std::move(feedback_selector), kp, ki, kd);

    builder.Connect(zero_source->get_output_port(),
                    controller_->get_control_input_port());

    builder.ExportInput(controller_->get_state_input_port());
    builder.ExportOutput(controller_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_plant() const { return *plant_; }
  int position_index() const { return position_index_; }
  int velocity_index() const { return velocity_index_; }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  PidControlledSystem<T>* controller_{nullptr};
  int position_index_{};
  int velocity_index_{};
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto model = builder.AddSystem<PidControlledSchunkWsg<double>>();

  const RigidBodyTree<double>& tree =
      model->get_plant().get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer =
      builder.AddSystem<DrakeVisualizer>(tree, &lcm);

  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", &lcm));
  auto trajectory_generator = builder.AddSystem<SchunkWsgTrajectoryGenerator>(
      tree.get_num_positions() + tree.get_num_velocities(),
      model->position_index());

  auto status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", &lcm));
  auto status_sender = builder.AddSystem<SchunkWsgStatusSender>(
      tree.get_num_positions() + tree.get_num_velocities(),
      model->position_index(), model->velocity_index());

  builder.Connect(command_sub->get_output_port(0),
                  trajectory_generator->get_command_input_port());
  builder.Connect(*trajectory_generator, *model);
  builder.Connect(*model, *visualizer);
  builder.Connect(*model, *status_sender);
  builder.Connect(model->get_output_port(0),
                  trajectory_generator->get_state_input_port());
  builder.Connect(*status_sender, *status_pub);
  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::schunk_wsg::DoMain();
}

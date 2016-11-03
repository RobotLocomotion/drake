/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for
/// the physical arm, this simulation communicates over LCM using the
/// lcmt_iiwa_status and lcmt_iiwa_command messages, It is intended to
/// be a direct replacement for the iiwa driver.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

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
using systems::SystemOutput;

const int kNumJoints = 7;

// TODO(sam.creasey) If this could be made slightly more generic, it
// could be a good wrapper/template for LCM controlled robots.
class IiwaCommandReceiver : public systems::LeafSystem<double> {
 public:
  IiwaCommandReceiver() {
    this->DeclareAbstractInputPort(systems::kContinuousSampling);
    this->DeclareOutputPort(systems::kVectorValued, kNumJoints,
                            systems::kContinuousSampling);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const {
    const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
    DRAKE_ASSERT(input != nullptr);
    const auto& command = input->GetValue<lcmt_iiwa_command>();
    auto output_vec = this->GetMutableOutputVector(output, 0);

    // If we're using a default constructed message (haven't received
    // a command yet), just return 0.
    if (command.num_joints == 0) {
      output_vec.fill(0);
    } else {
      for (int i = 0; i < command.num_joints; i++) {
        output_vec(i) = command.joint_position[i];
      }
    }

    // TODO(sam.creasey) Support torque control some day.
    DRAKE_ASSERT(command.num_torques == 0);
  }
};

// This system has two input ports.  Input port zero is for the
// current state of the plant and input port one for the most recently
// received command.
class IiwaStatusSender : public systems::LeafSystem<double> {
 public:
  IiwaStatusSender() {
    this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2,
                           systems::kContinuousSampling);
    this->DeclareInputPort(systems::kVectorValued, kNumJoints,
                           systems::kContinuousSampling);
    this->DeclareAbstractOutputPort(systems::kContinuousSampling);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    auto output = std::make_unique<systems::LeafSystemOutput<double>>();
    lcmt_iiwa_status msg{};
    msg.num_joints = kNumJoints;
    msg.joint_position_measured.resize(msg.num_joints, 0);
    msg.joint_position_commanded.resize(msg.num_joints, 0);
    msg.joint_position_ipo.resize(msg.num_joints, 0);
    msg.joint_torque_measured.resize(msg.num_joints, 0);
    msg.joint_torque_commanded.resize(msg.num_joints, 0);
    msg.joint_torque_external.resize(msg.num_joints, 0);

    output->get_mutable_ports()->emplace_back(
        std::make_unique<systems::OutputPort>(
            std::make_unique<systems::Value<lcmt_iiwa_status>>(msg)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    systems::AbstractValue* mutable_data = output->GetMutableData(0);
    lcmt_iiwa_status& status =
        mutable_data->GetMutableValue<lcmt_iiwa_status>();

    status.timestamp = context.get_time() * 1e3;
    const systems::BasicVector<double>* state =
        this->EvalVectorInput(context, 0);
    const systems::BasicVector<double>* command =
        this->EvalVectorInput(context, 1);
    for (int i = 0; i < kNumJoints; i++) {
      status.joint_position_measured[i] = state->GetAtIndex(i);
      status.joint_position_commanded[i] = command->GetAtIndex(i);
    }
  }
};

// TODO(sam.creasey) We should de-duplicate this with kuka_demo.cc.
// I'm holding off for now because I still need to investigate how to
// create the combined Schunk+iiwa system, and I'm not sure what
// allowances I'll need in the simulation setup to get that to work.
template<typename T>
class SimulatedKuka : public systems::Diagram<T> {
 public:
  SimulatedKuka() {
    this->set_name("SimulatedKuka");

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto rigid_body_tree = std::make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/urdf/iiwa14_no_collision.urdf",
        drake::systems::plants::joints::kFixed,
        nullptr /* weld to frame */, rigid_body_tree.get());

    AddGround(rigid_body_tree.get());

    DiagramBuilder<T> builder;

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    std::unique_ptr<RigidBodyPlant<T>> plant =
        std::make_unique<RigidBodyPlant<T>>(std::move(rigid_body_tree));
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

    // The iiwa's control protocol doesn't have any way to express the
    // desired velocity for the arm, so this simulation doesn't take
    // target velocities as an input.  The PidControlledSystem does
    // want target velocities to calculate the D term.  Since we don't
    // have any logic to calculate the desired target velocity (yet!)
    // set the D term (to stabilize the arm near the commanded
    // position) and feed a desired velocity vector of zero.
    auto zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
        Eigen::VectorXd::Zero(kNumJoints));
    auto input_mux = builder.template AddSystem<Multiplexer<T>>(
        std::vector<int>{kNumJoints, kNumJoints});
    builder.Connect(zero_source->get_output_port(),
                    input_mux->get_input_port(1));
    builder.Connect(input_mux->get_output_port(0),
                    controller_->get_input_port(1));

    gravity_compensator_ = builder.template AddSystem<GravityCompensator<T>>(
        plant_->get_rigid_body_tree());

    // Split the input state into two signals one with the positions and one
    // with the velocities.
    // For Kuka:
    // -  get_num_states() = 14
    // -  get_num_positions() = 7
    rbp_state_demux_ = builder.template AddSystem<Demultiplexer<T>>(
        plant_->get_num_states(), plant_->get_num_positions());
    builder.Connect(controller_->get_output_port(0),
                    rbp_state_demux_->get_input_port(0));

    // Connects the gravity compensator to the output generalized positions.
    builder.Connect(rbp_state_demux_->get_output_port(0),
                    gravity_compensator_->get_input_port(0));
    builder.Connect(gravity_compensator_->get_output_port(0),
                    controller_->get_input_port(0));

    builder.ExportInput(input_mux->get_input_port(0));
    builder.ExportOutput(controller_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

  void SetDefaultState(Context<T>* context) const {
    Context<T>* controller_context =
        this->GetMutableSubsystemContext(context, controller_);
    controller_->SetDefaultState(controller_context);

    Context<T>* plant_context =
        controller_->GetMutableSubsystemContext(controller_context, plant_);
    plant_->SetZeroConfiguration(plant_context);
  }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  PidControlledSystem<T>* controller_{nullptr};
  Demultiplexer<T>* rbp_state_demux_{nullptr};
  GravityCompensator<T>* gravity_compensator_{nullptr};
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto model = builder.AddSystem<SimulatedKuka<double>>();

  const RigidBodyTree& tree = model->get_kuka_plant().get_rigid_body_tree();
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
                  status_sender->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(0),
                  status_sender->get_input_port(1));
  builder.Connect(status_sender->get_output_port(0),
                  status_pub->get_input_port(0));
  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  // Zeroes the state and initializes controller state.
  auto model_context = sys->GetMutableSubsystemContext(
      simulator.get_mutable_context(), model);
  model->SetDefaultState(model_context);

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

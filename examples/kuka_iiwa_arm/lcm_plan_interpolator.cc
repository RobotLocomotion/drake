#include "drake/examples/kuka_iiwa_arm/lcm_plan_interpolator.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/demultiplexer.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using manipulation::planner::InterpolatorType;
using manipulation::planner::RobotPlanInterpolator;
using systems::DiagramBuilder;

LcmPlanInterpolator::LcmPlanInterpolator(const std::string& model_path,
                                         InterpolatorType interpolator_type) {
  DiagramBuilder<double> builder;

  // Add plan interpolation block.
  robot_plan_interpolator_ =
      builder.AddSystem<RobotPlanInterpolator>(model_path, interpolator_type);
  num_joints_ = robot_plan_interpolator_->plant().num_positions();

  // Add block to export a received iiwa status.
  auto status_receiver = builder.AddSystem<IiwaStatusReceiver>(num_joints_);
  input_port_iiwa_status_ =
      builder.ExportInput(status_receiver->get_input_port());

  // Add a demux block to pull out the positions from the position + velocity
  // vector
  auto target_demux =
      builder.AddSystem<systems::Demultiplexer>(num_joints_ * 2, num_joints_);
  target_demux->set_name("target_demux");

  // Add a block to convert the desired position vector to iiwa command.
  auto command_sender = builder.AddSystem<IiwaCommandSender>(num_joints_);
  command_sender->set_name("command_sender");

  // Export the inputs.
  input_port_iiwa_plan_ =
      builder.ExportInput(robot_plan_interpolator_->get_plan_input_port());

  // Export the output.
  output_port_iiwa_command_ =
      builder.ExportOutput(command_sender->get_output_port());

  // Connect the subsystems.
  builder.Connect(robot_plan_interpolator_->get_output_port(0),
                  target_demux->get_input_port(0));
  builder.Connect(target_demux->get_output_port(0),
                  command_sender->get_position_input_port());

  // Build the system.
  builder.BuildInto(this);
}

void LcmPlanInterpolator::Initialize(double plan_start_time,
                                     const VectorX<double>& q0,
                                     systems::Context<double>* context) const {
  robot_plan_interpolator_->Initialize(
      plan_start_time, q0,
      &this->GetMutableSubsystemContext(*robot_plan_interpolator_, context)
          .get_mutable_state());
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

#include "drake/manipulation/robot_bridge/robot_bridge.h"

#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

RobotBridge::RobotBridge(const multibody::MultibodyPlant<double>* plant,
                         const multibody::Frame<double>* tool_frame,
                         const double timestep) {
  systems::DiagramBuilder<double> builder;

  // Primitive selector
  const int num_q = plant->num_positions();
  primitive_selector_ = builder.AddSystem<MotionPrimitiveSelector>(timestep);

  position_switch_ = builder.AddSystem<systems::PortSwitch<double>>(num_q);
  torque_switch_ = builder.AddSystem<systems::PortSwitch<double>>(num_q);
  motion_summary_switch_ =
      builder.AddSystem<systems::PortSwitch<double>>(MotionSummary());

  // Add primitives.
  auto move_q = builder.AddSystem<MoveJoint>(plant, timestep);
  primitives_[move_q->get_name()] = move_q;
  auto move_t =
      builder.AddSystem<MoveToolStraight>(plant, tool_frame, timestep);
  primitives_[move_t->get_name()] = move_t;

  // Add a passthrough for state to all primitives.
  auto pass_through = builder.AddSystem<systems::PassThrough<double>>(
      plant->num_multibody_states());

  for (const auto& primitive_pair : primitives_) {
    const systems::InputPort<double>& selector_input =
        primitive_selector_->DeclareInputForMotionPrimitive(
            *primitive_pair.second);

    const systems::InputPort<double>& position_switch_input =
        position_switch_->DeclareInputPort(primitive_pair.first);
    const systems::InputPort<double>& torque_switch_input =
        torque_switch_->DeclareInputPort(primitive_pair.first);
    const systems::InputPort<double>& summary_switch_input =
        motion_summary_switch_->DeclareInputPort(primitive_pair.first);

    // Connect primitive selector.
    builder.Connect(primitive_pair.second->get_motion_summary_output(),
                    selector_input);

    // Connect all switches' inputs.
    builder.Connect(primitive_pair.second->get_position_output(),
                    position_switch_input);
    builder.Connect(primitive_pair.second->get_torque_output(),
                    torque_switch_input);
    builder.Connect(primitive_pair.second->get_motion_summary_output(),
                    summary_switch_input);

    // Connect state input to all primitives.
    builder.Connect(pass_through->get_output_port(),
                    primitive_pair.second->get_state_input());
  }

  // Connect the selector port.
  builder.Connect(primitive_selector_->get_selection_output(),
                  position_switch_->get_port_selector_input_port());
  builder.Connect(primitive_selector_->get_selection_output(),
                  torque_switch_->get_port_selector_input_port());
  builder.Connect(primitive_selector_->get_selection_output(),
                  motion_summary_switch_->get_port_selector_input_port());

  // Expose IO
  builder.ExportInput(pass_through->get_input_port(), "state");
  builder.ExportInput(move_q->get_trajectory_input(), "q_trajectory");
  builder.ExportInput(move_t->get_trajectory_input(), "tool_trajectory");

  builder.ExportOutput(motion_summary_switch_->get_output_port(),
                       "motion_summary");
  builder.ExportOutput(position_switch_->get_output_port(), "position");
  builder.ExportOutput(torque_switch_->get_output_port(), "torque");

  builder.BuildInto(this);
}

void RobotBridge::Initialize(const Eigen::VectorXd& x0,
                             systems::Context<double>* context) const {
  for (const auto& pair : primitives_) {
    const MotionPrimitive* primitive = pair.second;
    auto& subcontext = this->GetMutableSubsystemContext(*primitive, context);
    primitive->Initialize(x0, &subcontext);
  }
}

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake

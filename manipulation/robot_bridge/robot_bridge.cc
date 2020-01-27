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
  primitive_selector_ = builder.AddSystem<MotionPrimitiveSelector>(
      plant->num_positions(), timestep);

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
    // Connect between primitive selector and all primitives.
    const systems::InputPort<double>* summary;
    const systems::InputPort<double>* position;
    const systems::InputPort<double>* torque;
    std::tie(summary, position, torque) =
        primitive_selector_->DeclareInputs(*primitive_pair.second);

    builder.Connect(primitive_pair.second->get_motion_summary_output(),
                    *summary);
    builder.Connect(primitive_pair.second->get_position_output(), *position);
    builder.Connect(primitive_pair.second->get_torque_output(), *torque);

    // Connect state input to all primitives.
    builder.Connect(pass_through->get_output_port(),
                    primitive_pair.second->get_state_input());
  }
  primitive_selector_->Finalize();

  // Expose IO
  builder.ExportInput(pass_through->get_input_port(), "state");
  builder.ExportInput(move_q->get_trajectory_input(), "q_trajectory");
  builder.ExportInput(move_t->get_trajectory_input(), "tool_trajectory");

  builder.ExportOutput(primitive_selector_->get_motion_summary_output(),
                       "motion_summary");
  builder.ExportOutput(primitive_selector_->get_position_output(), "position");
  builder.ExportOutput(primitive_selector_->get_torque_output(), "torque");

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

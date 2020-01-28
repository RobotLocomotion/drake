#include "drake/manipulation/robot_bridge/motion_primitive_selector.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

MotionPrimitiveSelector::MotionPrimitiveSelector(double timestep) {
  this->DeclarePeriodicUnrestrictedUpdate(timestep, 0);

  active_primitive_name_index_ =
      this->DeclareAbstractState(AbstractValue::Make<std::string>("MoveJoint"));

  selection_output_port_ = &this->DeclareAbstractOutputPort(
      "motion_primitive_selection", systems::InputPortIndex(),
      &MotionPrimitiveSelector::CalcSelection,
      {this->abstract_state_ticket(active_primitive_name_index_)});
}

const systems::InputPort<double>&
MotionPrimitiveSelector::DeclareInputForMotionPrimitive(
    const MotionPrimitive& primitive) {
  const std::string& name = primitive.get_name();
  DRAKE_THROW_UNLESS(primitive_name_to_input_index_.count(name) == 0);
  const systems::InputPort<double>& summary_input =
      this->DeclareAbstractInputPort(primitive.get_name() + "_summary_input",
                                     Value<MotionSummary>());
  primitive_name_to_input_index_[name] = summary_input.get_index();

  primitive_name_to_state_index_[name] = this->DeclareAbstractState(
      AbstractValue::Make<MotionSummary>({name, MotionStatus::kDone}));

  return summary_input;
}

void MotionPrimitiveSelector::CalcSelection(
    const systems::Context<double>& context,
    systems::InputPortIndex* summary) const {
  const auto& active_primitive_name =
      context.get_abstract_state<std::string>(active_primitive_name_index_);
  // Since this output is going into a PortSwitch, it needs to skip dest's
  // selector port, which is port 0. Thus offset by 1.
  *summary = systems::InputPortIndex(
      primitive_name_to_input_index_.at(active_primitive_name) + 1);
  drake::log()->info("active primitive: {}, port idx: {}",
                     active_primitive_name, *summary);
}

void MotionPrimitiveSelector::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  std::optional<std::string> new_active_primitive;
  for (const auto& pair : primitive_name_to_input_index_) {
    const auto& current_motion_summary =
        this->get_input_port(pair.second).Eval<MotionSummary>(context);
    auto& prev_motion_summary =
        state->get_mutable_abstract_state<MotionSummary>(
            primitive_name_to_state_index_.at(pair.first));

    if (prev_motion_summary.status != MotionStatus::kExecuting &&
        current_motion_summary.status == MotionStatus::kExecuting) {
      drake::log()->info("{}: {} -> {}", pair.first, prev_motion_summary.status,
                         current_motion_summary.status);
      DRAKE_THROW_UNLESS(!new_active_primitive.has_value());
      new_active_primitive = pair.first;
    }

    prev_motion_summary = current_motion_summary;
  }

  if (new_active_primitive.has_value()) {
    auto& current_active_primitive =
        state->get_mutable_abstract_state<std::string>(
            active_primitive_name_index_);
    drake::log()->info("Switching primitive from {} to {} at t = {}",
                       current_active_primitive, new_active_primitive.value(),
                       context.get_time());
    current_active_primitive = new_active_primitive.value();
  }
}

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake

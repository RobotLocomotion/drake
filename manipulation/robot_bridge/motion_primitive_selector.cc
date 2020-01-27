#include "drake/manipulation/robot_bridge/motion_primitive_selector.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

MotionPrimitiveSelector::MotionPrimitiveSelector(int num_q, double timestep)
    : num_q_(num_q) {
  this->DeclarePeriodicUnrestrictedUpdate(timestep, 0);
}

void MotionPrimitiveSelector::Finalize() {
  DRAKE_THROW_UNLESS(!finalized_);

  active_primitive_name_index_ =
      this->DeclareAbstractState(AbstractValue::Make<std::string>("MoveJoint"));
  for (const auto& pair : primitive_name_to_input_index_) {
    primitive_name_to_state_index_[pair.first] = this->DeclareAbstractState(
        AbstractValue::Make<MotionSummary>({pair.first, MotionStatus::kDone}));
  }

  std::set<systems::DependencyTicket> motion_summary_dep, position_dep,
      torque_dep;
  motion_summary_dep.insert(
      this->abstract_state_ticket(active_primitive_name_index_));
  position_dep = motion_summary_dep;
  torque_dep = motion_summary_dep;

  auto add_ticket = [this](const systems::InputPort<double>& port,
                           std::set<systems::DependencyTicket>* tickets) {
    tickets->insert(this->input_port_ticket(port.get_index()));
  };

  for (const auto& pair : primitive_name_to_input_index_) {
    const std::string& primitive_name = pair.first;
    add_ticket(get_motion_summary_input(primitive_name), &motion_summary_dep);
    add_ticket(get_position_input(primitive_name), &position_dep);
    add_ticket(get_torque_input(primitive_name), &torque_dep);
  }

  summary_output_port_ = &this->DeclareAbstractOutputPort(
      "motion_summary", MotionSummary(),
      &MotionPrimitiveSelector::CalcMotionSummary, motion_summary_dep);
  const systems::BasicVector<double> temp_q(num_q_);
  position_output_port_ = &this->DeclareVectorOutputPort(
      "position", temp_q, &MotionPrimitiveSelector::CalcPositionOutput,
      position_dep);
  torque_output_port_ = &this->DeclareVectorOutputPort(
      "torque", temp_q, &MotionPrimitiveSelector::CalcTorqueOutput, torque_dep);

  finalized_ = true;
}

std::tuple<const systems::InputPort<double>*, const systems::InputPort<double>*,
           const systems::InputPort<double>*>
MotionPrimitiveSelector::DeclareInputs(const MotionPrimitive& primitive) {
  DRAKE_THROW_UNLESS(
      primitive_name_to_input_index_.count(primitive.get_name()) == 0);
  const systems::BasicVector<double> temp_q(num_q_);
  const systems::InputPort<double>* summary_input =
      &this->DeclareAbstractInputPort(primitive.get_name() + "_summary_input",
                                      Value<MotionSummary>());
  const systems::InputPort<double>* position_input =
      &this->DeclareVectorInputPort(primitive.get_name() + "_position_input",
                                    temp_q);
  const systems::InputPort<double>* torque_input =
      &this->DeclareVectorInputPort(primitive.get_name() + "_torque_input",
                                    temp_q);
  primitive_name_to_input_index_[primitive.get_name()] =
      summary_input->get_index();
  return std::make_tuple(summary_input, position_input, torque_input);
}

const systems::InputPort<double>&
MotionPrimitiveSelector::get_motion_summary_input(
    const std::string& primitive_name) const {
  DRAKE_THROW_UNLESS(primitive_name_to_input_index_.count(primitive_name) != 0);
  return this->get_input_port(
      primitive_name_to_input_index_.at(primitive_name));
}

const systems::InputPort<double>& MotionPrimitiveSelector::get_position_input(
    const std::string& primitive_name) const {
  DRAKE_THROW_UNLESS(primitive_name_to_input_index_.count(primitive_name) != 0);
  return this->get_input_port(
      primitive_name_to_input_index_.at(primitive_name) + 1);
}

const systems::InputPort<double>& MotionPrimitiveSelector::get_torque_input(
    const std::string& primitive_name) const {
  DRAKE_THROW_UNLESS(primitive_name_to_input_index_.count(primitive_name) != 0);
  return this->get_input_port(
      primitive_name_to_input_index_.at(primitive_name) + 2);
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

void MotionPrimitiveSelector::CalcTorqueOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const auto& active_primitive_name =
      context.get_abstract_state<std::string>(active_primitive_name_index_);
  const auto& port = get_torque_input(active_primitive_name);
  output->set_value(
      this->EvalVectorInput(context, port.get_index())->get_value());
}

void MotionPrimitiveSelector::CalcPositionOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const auto& active_primitive_name =
      context.get_abstract_state<std::string>(active_primitive_name_index_);
  const auto& port = get_position_input(active_primitive_name);
  output->set_value(
      this->EvalVectorInput(context, port.get_index())->get_value());
}

void MotionPrimitiveSelector::CalcMotionSummary(
    const systems::Context<double>& context, MotionSummary* summary) const {
  const auto& active_primitive_name =
      context.get_abstract_state<std::string>(active_primitive_name_index_);
  const auto& port = get_motion_summary_input(active_primitive_name);
  *summary = port.Eval<MotionSummary>(context);
}

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake

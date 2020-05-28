#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/lcm_messages.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

namespace {
std::unique_ptr<AbstractValue> MakeCommandMessage() {
  return std::make_unique<Value<lcmt_iiwa_command>>();
}
}  // namespace

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints)
    : num_joints_(num_joints) {
  // Our parameter stores the position when no message has been received.
  const BasicVector<double> default_position(VectorXd::Zero(num_joints));
  const systems::NumericParameterIndex param{
      DeclareNumericParameter(default_position)};
  DRAKE_DEMAND(param == 0);  // We're depending on that elsewhere.

  DeclareAbstractInputPort("lcmt_iiwa_command", *MakeCommandMessage());
  DeclareInputPort("position_measured", systems::kVectorValued, num_joints_);
  groomed_input_ = &DeclareCacheEntry(
      "groomed_input", &IiwaCommandReceiver::CalcInput,
      {all_input_ports_ticket(), numeric_parameter_ticket(param)});

  DeclareVectorOutputPort(
      "position", BasicVector<double>(num_joints),
      [this](const Context<double>& context, BasicVector<double>* output) {
        output->SetFromVector(this->input_position(context));
      });
  DeclareVectorOutputPort(
      "torque", BasicVector<double>(num_joints),
      [this](const Context<double>& context, BasicVector<double>* output) {
        output->SetFromVector(this->input_torque(context));
      });
}

using InPort = systems::InputPort<double>;
const InPort& IiwaCommandReceiver::get_message_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& IiwaCommandReceiver::get_position_measured_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
using OutPort = systems::OutputPort<double>;
const OutPort& IiwaCommandReceiver::get_commanded_position_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& IiwaCommandReceiver::get_commanded_torque_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorXd>& q) const {
  DRAKE_THROW_UNLESS(q.size() == num_joints_);
  context->get_mutable_numeric_parameter(0).SetFromVector(q);
}

// Returns (in "result") the command message input, or if a message has not
// been received yet returns the a fallback value.  The result always has
// num_joints_ positions and torques.
void IiwaCommandReceiver::CalcInput(
  const Context<double>& context, lcmt_iiwa_command* result) const {
  if (!get_message_input_port().HasValue(context)) {
    throw std::logic_error("IiwaCommandReceiver has no input connected");
  }

  // Copies the input value into our tentative result.
  *result = get_message_input_port().Eval<lcmt_iiwa_command>(context);

  // If we haven't received a non-default message yet, use nominal values
  // instead.  N.B. This works due to lcm::Serializer<>::CreateDefaultValue()
  // using value-initialization.
  if (lcm::AreLcmMessagesEqual(*result, lcmt_iiwa_command{})) {
    const VectorXd positions =
        get_position_measured_input_port().HasValue(context) ?
            get_position_measured_input_port().Eval(context) :
            context.get_numeric_parameter(0).get_value();
    result->num_joints = positions.size();
    result->joint_position =
        {positions.data(), positions.data() + positions.size()};
  }

  // Sanity check the joint sizes.  If torques were not sent, pad with zeros.
  if (result->num_joints != num_joints_) {
    throw std::runtime_error(fmt::format(
        "IiwaCommandReceiver expected num_joints = {}, but received {}",
        num_joints_, result->num_joints));
  }
  if (result->num_torques == 0) {
    result->num_torques = num_joints_;
    result->joint_torque.resize(num_joints_, 0.0);
  } else if (result->num_torques != num_joints_) {
    throw std::runtime_error(fmt::format(
        "IiwaCommandReceiver expected num_torques = {}, but received {}",
        num_joints_, result->num_torques));
  }
}

IiwaCommandReceiver::MapVectorXd IiwaCommandReceiver::input_position(
    const Context<double>& context) const {
  const auto& message = groomed_input_->Eval<lcmt_iiwa_command>(context);
  return Eigen::Map<const VectorXd>(
      message.joint_position.data(),
      message.joint_position.size());
}

IiwaCommandReceiver::MapVectorXd IiwaCommandReceiver::input_torque(
    const Context<double>& context) const {
  const auto& message = groomed_input_->Eval<lcmt_iiwa_command>(context);
  return Eigen::Map<const VectorXd>(
      message.joint_torque.data(),
      message.joint_torque.size());
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

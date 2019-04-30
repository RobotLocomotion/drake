#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

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

  // Our input ports are mutually exclusive; exactly one connected input port
  // feeds our cache entry. The computation may be dependent on the above
  // parameter as well.
  DeclareAbstractInputPort("lcmt_iiwa_command", *MakeCommandMessage());
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

const systems::InputPort<double>& IiwaCommandReceiver::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
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
// been received yet returns the initial command (as optionally set by the
// user).  The result will always have have num_joints_ positions and torques.
void IiwaCommandReceiver::CalcInput(
  const Context<double>& context, lcmt_iiwa_command* result) const {
  if (!get_input_port().HasValue(context)) {
    throw std::logic_error("IiwaCommandReceiver has no input connected");
  }

  // Copies the (sole) input value, converting from IiwaCommand if necessary.
  *result = get_input_port().Eval<lcmt_iiwa_command>(context);

  // If we haven't received a legit message yet, use the initial command.
  if (result->utime == 0.0) {
    const VectorXd param = context.get_numeric_parameter(0).get_value();
    result->num_joints = param.size();
    result->joint_position = {param.data(), param.data() + param.size()};
    result->num_torques = 0;
    result->joint_torque.clear();
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

std::unique_ptr<systems::lcm::LcmSubscriberSystem>
MakeIiwaCommandLcmSubscriberSystem(
    int, const std::string& channel,
    drake::lcm::DrakeLcmInterface* lcm) {
  return systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
      channel, lcm);
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

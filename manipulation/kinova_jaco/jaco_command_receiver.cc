#include "drake/manipulation/kinova_jaco/jaco_command_receiver.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/lcm_messages.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

namespace {
std::unique_ptr<AbstractValue> MakeCommandMessage() {
  return std::make_unique<Value<lcmt_jaco_command>>();
}
}  // namespace

JacoCommandReceiver::JacoCommandReceiver(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  // Our parameters store the position when no message has been received.
  const BasicVector<double> default_position(
      VectorXd::Zero(num_joints + num_fingers));
  const systems::NumericParameterIndex arm_param{
      DeclareNumericParameter(default_position)};
  DRAKE_DEMAND(arm_param == 0);  // We're depending on that elsewhere.

  // Our input ports are mutually exclusive; exactly one connected input port
  // feeds our cache entry. The computation may be dependent on the above
  // parameter as well.
  DeclareAbstractInputPort("lcmt_jaco_command", *MakeCommandMessage());
  groomed_input_ = &DeclareCacheEntry(
      "groomed_input", &JacoCommandReceiver::CalcInput,
      {all_input_ports_ticket(), numeric_parameter_ticket(arm_param), });

  DeclareVectorOutputPort(
      "state", (num_joints + num_fingers) * 2,
      [this](const Context<double>& context, BasicVector<double>* output) {
        output->SetFromVector(this->input_state(context));
      });
}

void JacoCommandReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorXd>& q) const {
  DRAKE_THROW_UNLESS(q.size() == num_joints_ + num_fingers_);
  context->get_mutable_numeric_parameter(0).SetFromVector(q);
}

// Returns (in "result") the command message input, or if a message has not
// been received yet returns the initial command (as optionally set by the
// user).  The result will always have num_joints_ positions and torques.
void JacoCommandReceiver::CalcInput(
  const Context<double>& context, lcmt_jaco_command* result) const {
  if (!get_input_port().HasValue(context)) {
    throw std::logic_error("JacoCommandReceiver has no input connected");
  }

  // Copies the (sole) input value, converting from JacoCommand if necessary.
  *result = get_input_port().Eval<lcmt_jaco_command>(context);

  // If we haven't received a non-default message yet, use the initial command.
  // N.B. This works due to lcm::Serializer<>::CreateDefaultValue() using
  // value-initialization.
  if (lcm::AreLcmMessagesEqual(*result, lcmt_jaco_command{})) {
    const VectorXd arm_param = context.get_numeric_parameter(0).get_value();
    result->num_joints = num_joints_;
    result->joint_position =
        {arm_param.data(), arm_param.data() + num_joints_};
    result->joint_velocity.resize(num_joints_, 0);

    result->num_fingers = num_fingers_;
    if (num_fingers_) {
      result->finger_position =
          {arm_param.data() + num_joints_,
           arm_param.data() + num_joints_ + num_fingers_};
      result->finger_velocity.resize(num_fingers_, 0);
    } else {
      result->finger_position.clear();
      result->finger_velocity.clear();
    }
  } else {
    for (int i = 0; i < result->num_fingers; ++i) {
      result->finger_position[i] *= kFingerSdkToUrdf;
      result->finger_velocity[i] *= kFingerSdkToUrdf;
    }
  }

  // Sanity check the joint sizes.
  if (result->num_joints != num_joints_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_joints = {}, but received {}",
        num_joints_, result->num_joints));
  }

  if (result->num_fingers != num_fingers_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_fingers = {}, but received {}",
        num_fingers_, result->num_fingers));
  }
}

Eigen::VectorXd JacoCommandReceiver::input_state(
    const Context<double>& context) const {
  const auto& message = groomed_input_->Eval<lcmt_jaco_command>(context);
  Eigen::VectorXd state((num_joints_ + num_fingers_) * 2);
  state.head(num_joints_) = Eigen::Map<const VectorXd>(
      message.joint_position.data(), message.joint_position.size());
  if (num_fingers_) {
    state.segment(num_joints_, num_fingers_) = Eigen::Map<const VectorXd>(
        message.finger_position.data(), message.finger_position.size());
  }
  state.segment(num_joints_ + num_fingers_, num_joints_) =
      Eigen::Map<const VectorXd>(
          message.joint_velocity.data(), message.joint_velocity.size());
  if (num_fingers_) {
    state.tail(num_fingers_) = Eigen::Map<const VectorXd>(
      message.finger_velocity.data(), message.finger_velocity.size());
  }
  return state;
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake

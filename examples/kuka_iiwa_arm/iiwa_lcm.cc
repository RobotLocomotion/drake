#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/rigid_body_plant/contact_results.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::State;
using systems::SystemOutput;
using systems::DiscreteUpdateEvent;

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the iiwa's control
// cabinet.
const double kIiwaLcmStatusPeriod = 0.005;

template <typename T>
IiwaCommand<T>::IiwaCommand(int num_joints)
    : systems::BasicVector<T>(num_joints * 2 + 1), num_joints_(num_joints) {
  this->values().setZero();
  set_utime(kUnitializedTime);
}

template <typename T>
T IiwaCommand<T>::utime() const {
  return (*this)[0];
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> IiwaCommand<T>::joint_position() const {
  return this->values().segment(1, num_joints_);
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> IiwaCommand<T>::joint_torque() const {
  return this->values().tail(num_joints_);
}

template <typename T>
void IiwaCommand<T>::set_utime(T utime) {
  (*this)[0] = utime;
}

template <typename T>
void IiwaCommand<T>::set_joint_position(const VectorX<T>& q) {
  DRAKE_THROW_UNLESS(q.size() == num_joints_);
  this->values().segment(1, num_joints_) = q;
}

template <typename T>
void IiwaCommand<T>::set_joint_torque(const VectorX<T>& torque) {
  DRAKE_THROW_UNLESS(torque.size() == num_joints_);
  this->values().tail(num_joints_) = torque;
}

IiwaCommandTranslator::IiwaCommandTranslator(int num_joints)
    : systems::lcm::LcmAndVectorBaseTranslator(
          IiwaCommand<double>(num_joints).size()),
      num_joints_(num_joints) {}

void IiwaCommandTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  auto command = dynamic_cast<IiwaCommand<double>*>(vector_base);
  DRAKE_THROW_UNLESS(command);

  lcmt_iiwa_command msg{};
  const int length = msg.decode(lcm_message_bytes, 0, lcm_message_length);
  DRAKE_THROW_UNLESS(length == lcm_message_length);

  const int num_joints = static_cast<int>(msg.joint_position.size());
  DRAKE_THROW_UNLESS(num_joints == num_joints_);
  Eigen::VectorXd q(num_joints);
  for (int i = 0; i < num_joints; i++) q[i] = msg.joint_position[i];

  const int num_torques = static_cast<int>(msg.joint_torque.size());
  DRAKE_THROW_UNLESS(num_torques == 0 || num_torques == num_joints_);
  Eigen::VectorXd torque = Eigen::VectorXd::Zero(num_joints_);
  if (num_torques) {
    for (int i = 0; i < num_joints_; i++) torque[i] = msg.joint_torque[i];
  }

  command->set_utime(msg.utime);
  command->set_joint_position(q);
  command->set_joint_torque(torque);
}

void IiwaCommandTranslator::Serialize(double,
                                      const systems::VectorBase<double>&,
                                      std::vector<uint8_t>*) const {
  throw std::runtime_error("Not implemented");
}

std::unique_ptr<systems::BasicVector<double>>
IiwaCommandTranslator::AllocateOutputVector() const {
  return std::make_unique<IiwaCommand<double>>(num_joints_);
}

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints)
    : num_joints_(num_joints), translator_(num_joints) {
  lcmt_iiwa_command uninitialized_message{};
  uninitialized_message.utime =
      static_cast<uint64_t>(IiwaCommand<double>::kUnitializedTime);
  this->DeclareVectorInputPort("command_vector",
                               IiwaCommand<double>(num_joints));
  this->DeclareAbstractInputPort(
      "command_message",
      systems::Value<lcmt_iiwa_command>(uninitialized_message));

  this->DeclareVectorOutputPort(
      "state", systems::BasicVector<double>(num_joints_ * 2),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 0, num_joints_ * 2, o);
      });
  this->DeclareVectorOutputPort(
      "feedforward_torque", systems::BasicVector<double>(num_joints_),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, num_joints_ * 2, num_joints_, o);
      });
  this->DeclarePeriodicDiscreteUpdate(kIiwaLcmStatusPeriod);
  // State + torque
  this->DeclareDiscreteState(num_joints_ * 3);
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.setZero();
  state_value.head(num_joints_) = x;
}

void IiwaCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const IiwaCommand<double>* command =
      this->template EvalVectorInput<IiwaCommand>(context, 0);

  // If the vector input port is not wired, try the abstract value one.
  IiwaCommand<double> decoded_command(num_joints_);
  if (!command) {
    const systems::AbstractValue* input = this->EvalAbstractInput(context, 1);
    DRAKE_THROW_UNLESS(input != nullptr);
    const auto& command_msg = input->GetValue<lcmt_iiwa_command>();
    std::vector<uint8_t> bytes(command_msg.getEncodedSize());
    command_msg.encode(bytes.data(), 0, bytes.size());
    translator_.Deserialize(bytes.data(), bytes.size(), &decoded_command);
    command = &decoded_command;
  }
  DRAKE_THROW_UNLESS(command);

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  // Haven't received a legit message yet.
  if (command->utime() == IiwaCommand<double>::kUnitializedTime) {
    return;
  }

  state_value.segment(num_joints_, num_joints_) =
      (command->joint_position() - state_value.head(num_joints_)) /
      kIiwaLcmStatusPeriod;
  state_value.head(num_joints_) = command->joint_position();
  state_value.tail(num_joints_) = command->joint_torque();
}

void IiwaCommandReceiver::CopyStateToOutput(const Context<double>& context,
                                            int start_idx, int length,
                                            BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec =
      context.get_discrete_state(0).get_value().segment(start_idx, length);
}

IiwaCommandSender::IiwaCommandSender(int num_joints)
    : num_joints_(num_joints),
      position_input_port_(
          this->DeclareInputPort(systems::kVectorValued, num_joints_)
              .get_index()),
      torque_input_port_(
          this->DeclareInputPort(systems::kVectorValued, num_joints_)
              .get_index()) {
  this->DeclareAbstractOutputPort(&IiwaCommandSender::OutputCommand);
}

void IiwaCommandSender::OutputCommand(const Context<double>& context,
                                      lcmt_iiwa_command* output) const {
  lcmt_iiwa_command& command = *output;

  command.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* positions =
      this->EvalVectorInput(context, 0);

  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = positions->GetAtIndex(i);
  }

  const systems::BasicVector<double>* torques =
      this->EvalVectorInput(context, 1);
  if (torques == nullptr) {
    command.num_torques = 0;
    command.joint_torque.clear();
  } else {
    command.num_torques = num_joints_;
    command.joint_torque.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i) {
      command.joint_torque[i] = torques->GetAtIndex(i);
    }
  }
}

IiwaStatusReceiver::IiwaStatusReceiver(int num_joints)
    : num_joints_(num_joints),
      position_commanded_output_port_(
          this->DeclareVectorOutputPort(
                  "position_commanded",
                  systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_position_commanded>)
              .get_index()),
      position_measured_output_port_(
          this->DeclareVectorOutputPort(
                  "position_measured",
                  systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_position_measured>)
              .get_index()),
      velocity_estimated_output_port_(
          this->DeclareVectorOutputPort(
                  "velocity_estimated",
                  systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_velocity_estimated>)
              .get_index()),
      torque_commanded_output_port_(
          this->DeclareVectorOutputPort(
                  "torque_commanded", systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_torque_commanded>)
              .get_index()),
      torque_measured_output_port_(
          this->DeclareVectorOutputPort(
                  "torque_measured", systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_torque_measured>)
              .get_index()),
      torque_external_output_port_(
          this->DeclareVectorOutputPort(
                  "torque_external", systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_torque_external>)
              .get_index()),
      state_output_port_(
          this->DeclareVectorOutputPort(
                  "state",
                  systems::BasicVector<double>(num_joints_ * 2),
                  &IiwaStatusReceiver::OutputState)
              .get_index()),
      deprecated_measured_position_output_port_(
          this->DeclareVectorOutputPort(
                  "deprecated_measured_position",
                  systems::BasicVector<double>(num_joints_ * 3),
                  &IiwaStatusReceiver::OutputDeprecatedMeasuredPosition)
              .get_index()) {
  this->DeclareAbstractInputPort(
      "lcmt_iiwa_status",
      systems::Value<lcmt_iiwa_status>{});
}

template <std::vector<double> drake::lcmt_iiwa_status::* field>
void IiwaStatusReceiver::CopyLcmVectorOut(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    Eigen::VectorBlock<VectorX<double>> output_vec =
    output->get_mutable_value();
    for (int i = 0; i < num_joints_; ++i) {
      output_vec(i) = (status.*field)[i];
    }
  }
}

void IiwaStatusReceiver::OutputState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    Eigen::VectorBlock<VectorX<double>> output_vec =
        output->get_mutable_value();
    for (int i = 0; i < num_joints_; ++i) {
      output_vec(i) = status.joint_position_measured[i];
      output_vec(i + num_joints_) = status.joint_velocity_estimated[i];
    }
  }
}

void IiwaStatusReceiver::OutputDeprecatedMeasuredPosition(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    Eigen::VectorBlock<VectorX<double>> lumped_output =
        output->get_mutable_value();
    for (int i = 0; i < num_joints_; ++i) {
      lumped_output(i) = status.joint_position_measured[i];
      lumped_output(i + num_joints_) = status.joint_velocity_estimated[i];
      lumped_output(i + 2 * num_joints_) = status.joint_position_commanded[i];
    }
  }
}

IiwaStatusSender::IiwaStatusSender(int num_joints) : num_joints_(num_joints) {
  // Commanded state.
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  // Measured state.
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  // Commanded torque.
  this->DeclareInputPort(systems::kVectorValued, num_joints_);
  // Measured torque.
  this->DeclareInputPort(systems::kVectorValued, num_joints_);
  // Measured external torque.
  this->DeclareInputPort(systems::kVectorValued, num_joints_);

  this->DeclareAbstractOutputPort(&IiwaStatusSender::MakeOutputStatus,
                                  &IiwaStatusSender::OutputStatus);
}

lcmt_iiwa_status IiwaStatusSender::MakeOutputStatus() const {
  lcmt_iiwa_status msg{};
  msg.num_joints = num_joints_;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_velocity_estimated.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);
  return msg;
}

void IiwaStatusSender::OutputStatus(const Context<double>& context,
                                    lcmt_iiwa_status* output) const {
  lcmt_iiwa_status& status = *output;

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* state = this->EvalVectorInput(context, 1);
  const systems::BasicVector<double>* commanded_torque =
      this->EvalVectorInput(context, 2);
  const systems::BasicVector<double>* measured_torque =
      this->EvalVectorInput(context, 3);
  const systems::BasicVector<double>* external_torque =
      this->EvalVectorInput(context, 4);

  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_velocity_estimated[i] = state->GetAtIndex(i + num_joints_);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
    status.joint_torque_commanded[i] = commanded_torque->GetAtIndex(i);

    if (external_torque) {
      status.joint_torque_external[i] = external_torque->GetAtIndex(i);
    }
    if (measured_torque) {
      status.joint_torque_measured[i] = measured_torque->GetAtIndex(i);
    } else {
      // TODO(rcory) Update joint_torque_measured to report actual measured
      // torque once RigidBodyPlant supports it. For now, assume
      // joint_torque_measured == joint_torque_commanded.
      status.joint_torque_measured[i] = commanded_torque->GetAtIndex(i);
    }
  }
}

IiwaContactResultsToExternalTorque::IiwaContactResultsToExternalTorque(
    const RigidBodyTree<double>& tree,
    const std::vector<int>& model_instance_ids)
    : num_joints_{tree.get_num_velocities()} {
  int length = 0;
  velocity_map_.resize(tree.get_num_model_instances(),
                       std::pair<int, int>(-1, -1));

  for (const auto& body : tree.get_bodies()) {
    if (!body->has_parent_body()) {
      continue;
    }

    const int instance_id = body->get_model_instance_id();
    const int velocity_start_index = body->get_velocity_start_index();
    const int num_velocities = body->getJoint().get_num_velocities();

    if (std::find(model_instance_ids.begin(), model_instance_ids.end(),
                  instance_id) == model_instance_ids.end()) {
      continue;
    }

    if (num_velocities) {
      if (velocity_map_[instance_id].first == -1) {
        velocity_map_[instance_id] =
            std::pair<int, int>(velocity_start_index, num_velocities);
      } else {
        std::pair<int, int> map_entry = velocity_map_[instance_id];
        DRAKE_DEMAND(velocity_start_index ==
                     map_entry.first + map_entry.second);
        map_entry.second += num_velocities;
        velocity_map_[instance_id] = map_entry;
      }
      length += num_velocities;
    }
  }

  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      systems::Value<systems::ContactResults<double>>{});

  this->DeclareVectorOutputPort(
      systems::BasicVector<double>(length),
      &IiwaContactResultsToExternalTorque::OutputExternalTorque);
}

void IiwaContactResultsToExternalTorque::OutputExternalTorque(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const systems::ContactResults<double>* contact_results =
      this->EvalInputValue<systems::ContactResults<double>>(context, 0);

  int start = 0;
  const VectorX<double>& generalized_force =
      contact_results->get_generalized_contact_force();
  DRAKE_DEMAND(generalized_force.size() == num_joints_);

  for (const auto& entry : velocity_map_) {
    const int v_idx = entry.first;
    const int v_length = entry.second;
    if (v_idx == -1) continue;

    for (int idx = 0; idx < v_length; idx++) {
      output->SetAtIndex(start + idx, generalized_force[v_idx + idx]);
    }
    start += v_length;
  }
  DRAKE_DEMAND(start == output->size());
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

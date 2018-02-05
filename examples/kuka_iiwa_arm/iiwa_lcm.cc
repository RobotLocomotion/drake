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

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(
      systems::BasicVector<double>(num_joints_ * 2),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 0, num_joints_ * 2, o);
      });
  this->DeclareVectorOutputPort(
      systems::BasicVector<double>(num_joints_),
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
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_iiwa_command>();

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (command.num_joints != 0) {
    DRAKE_DEMAND(command.num_joints == num_joints_);
    VectorX<double> new_positions(num_joints_);
    for (int i = 0; i < command.num_joints; ++i) {
      new_positions(i) = command.joint_position[i];
    }

    state_value.segment(num_joints_, num_joints_) =
        (new_positions - state_value.head(num_joints_)) / kIiwaLcmStatusPeriod;
    state_value.head(num_joints_) = new_positions;
  }

  // If the message does not contain torque commands, set torque command to
  // zeros.
  if (command.num_torques == 0) {
    state_value.tail(num_joints_).setZero();
  } else {
    DRAKE_DEMAND(command.num_torques == num_joints_);
    for (int i = 0; i < num_joints_; i++)
      state_value[2 * num_joints_ + i] = command.joint_torque[i];
  }
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
      measured_position_output_port_(
          this->DeclareVectorOutputPort(
                  systems::BasicVector<double>(num_joints_ * 2),
                  &IiwaStatusReceiver::OutputMeasuredPosition)
              .get_index()),
      commanded_position_output_port_(
          this->DeclareVectorOutputPort(
                  systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::OutputCommandedPosition)
              .get_index()) {
  this->DeclareAbstractInputPort();
  this->DeclareDiscreteState(num_joints_ * 3);
  this->DeclarePeriodicDiscreteUpdate(kIiwaLcmStatusPeriod);
}

void IiwaStatusReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), keep using the initial state.
  if (status.num_joints != 0) {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    VectorX<double> measured_position(num_joints_);
    VectorX<double> estimated_velocity(num_joints_);
    VectorX<double> commanded_position(num_joints_);
    for (int i = 0; i < status.num_joints; ++i) {
      measured_position(i) = status.joint_position_measured[i];
      estimated_velocity(i) = status.joint_velocity_estimated[i];
      commanded_position(i) = status.joint_position_commanded[i];
    }

    BasicVector<double>& state = discrete_state->get_mutable_vector(0);
    auto state_value = state.get_mutable_value();
    state_value.head(num_joints_) = measured_position;
    state_value.segment(num_joints_, num_joints_) = estimated_velocity;
    state_value.tail(num_joints_) = commanded_position;
  }
}

void IiwaStatusReceiver::OutputMeasuredPosition(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto state_value = context.get_discrete_state(0).get_value();

  Eigen::VectorBlock<VectorX<double>> measured_position_output =
      output->get_mutable_value();
  measured_position_output = state_value.head(num_joints_ * 2);
}

void IiwaStatusReceiver::OutputCommandedPosition(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto state_value = context.get_discrete_state(0).get_value();

  Eigen::VectorBlock<VectorX<double>> commanded_position_output =
      output->get_mutable_value();
  commanded_position_output = state_value.tail(num_joints_);
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

  for (const auto& body : tree.bodies) {
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

  this->DeclareAbstractInputPort();

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

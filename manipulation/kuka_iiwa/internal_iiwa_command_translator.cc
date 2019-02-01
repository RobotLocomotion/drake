#include "drake/manipulation/kuka_iiwa/internal_iiwa_command_translator.h"

#include <stdexcept>

#include "drake/common/drake_throw.h"
#include "drake/lcmt_iiwa_command.hpp"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace internal {

using systems::BasicVector;
using systems::VectorBase;

template <typename T>
IiwaCommand<T>::IiwaCommand(int num_joints)
    : BasicVector<T>(num_joints * 2 + 1), num_joints_(num_joints) {
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
IiwaCommandTranslator::IiwaCommandTranslator(int num_joints)
    : systems::lcm::LcmAndVectorBaseTranslator(
          IiwaCommand<double>(num_joints).size()),
      num_joints_(num_joints) {}
#pragma GCC diagnostic pop

void IiwaCommandTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
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
                                      const VectorBase<double>&,
                                      std::vector<uint8_t>*) const {
  throw std::runtime_error("Not implemented");
}

std::unique_ptr<BasicVector<double>>
IiwaCommandTranslator::AllocateOutputVector() const {
  return std::make_unique<IiwaCommand<double>>(num_joints_);
}

template class IiwaCommand<double>;

}  // namespace internal
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

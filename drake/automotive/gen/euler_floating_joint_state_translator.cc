#include "drake/automotive/gen/euler_floating_joint_state_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
EulerFloatingJointStateTranslator::AllocateOutputVector() const {
  return std::make_unique<EulerFloatingJointState<double>>();
}

void EulerFloatingJointStateTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const EulerFloatingJointState<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_euler_floating_joint_state_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.x = vector->x();
  message.y = vector->y();
  message.z = vector->z();
  message.roll = vector->roll();
  message.pitch = vector->pitch();
  message.yaw = vector->yaw();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void EulerFloatingJointStateTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector =
      dynamic_cast<EulerFloatingJointState<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_euler_floating_joint_state_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message euler_floating_joint_state.");
  }
  my_vector->set_x(message.x);
  my_vector->set_y(message.y);
  my_vector->set_z(message.z);
  my_vector->set_roll(message.roll);
  my_vector->set_pitch(message.pitch);
  my_vector->set_yaw(message.yaw);
}

}  // namespace automotive
}  // namespace drake

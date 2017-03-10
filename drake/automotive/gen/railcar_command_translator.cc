#include "drake/automotive/gen/railcar_command_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
RailcarCommandTranslator::AllocateOutputVector() const {
  return std::make_unique<RailcarCommand<double>>();
}

void RailcarCommandTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const RailcarCommand<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_railcar_command_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.throttle = vector->throttle();
  message.brake = vector->brake();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void RailcarCommandTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<RailcarCommand<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_railcar_command_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message railcar_command.");
  }
  my_vector->set_throttle(message.throttle);
  my_vector->set_brake(message.brake);
}

}  // namespace automotive
}  // namespace drake

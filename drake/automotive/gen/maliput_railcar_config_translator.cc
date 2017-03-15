#include "drake/automotive/gen/maliput_railcar_config_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
MaliputRailcarConfigTranslator::AllocateOutputVector() const {
  return std::make_unique<MaliputRailcarConfig<double>>();
}

void MaliputRailcarConfigTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const MaliputRailcarConfig<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_maliput_railcar_config_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.r = vector->r();
  message.h = vector->h();
  message.initial_speed = vector->initial_speed();
  message.max_speed = vector->max_speed();
  message.velocity_limit_kp = vector->velocity_limit_kp();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void MaliputRailcarConfigTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector =
      dynamic_cast<MaliputRailcarConfig<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_maliput_railcar_config_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message maliput_railcar_config.");
  }
  my_vector->set_r(message.r);
  my_vector->set_h(message.h);
  my_vector->set_initial_speed(message.initial_speed);
  my_vector->set_max_speed(message.max_speed);
  my_vector->set_velocity_limit_kp(message.velocity_limit_kp);
}

}  // namespace automotive
}  // namespace drake

#include "drake/automotive/gen/maliput_railcar_params_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
MaliputRailcarParamsTranslator::AllocateOutputVector() const {
  return std::make_unique<MaliputRailcarParams<double>>();
}

void MaliputRailcarParamsTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const MaliputRailcarParams<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_maliput_railcar_params_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.r = vector->r();
  message.h = vector->h();
  message.max_speed = vector->max_speed();
  message.velocity_limit_kp = vector->velocity_limit_kp();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void MaliputRailcarParamsTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector =
      dynamic_cast<MaliputRailcarParams<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_maliput_railcar_params_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message maliput_railcar_params.");
  }
  my_vector->set_r(message.r);
  my_vector->set_h(message.h);
  my_vector->set_max_speed(message.max_speed);
  my_vector->set_velocity_limit_kp(message.velocity_limit_kp);
}

}  // namespace automotive
}  // namespace drake

#include "drake/automotive/gen/endless_road_car_state_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
EndlessRoadCarStateTranslator::AllocateOutputVector() const {
  return std::make_unique<EndlessRoadCarState<double>>();
}

void EndlessRoadCarStateTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const EndlessRoadCarState<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_endless_road_car_state_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.s = vector->s();
  message.r = vector->r();
  message.heading = vector->heading();
  message.speed = vector->speed();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void EndlessRoadCarStateTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector =
      dynamic_cast<EndlessRoadCarState<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_endless_road_car_state_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message endless_road_car_state.");
  }
  my_vector->set_s(message.s);
  my_vector->set_r(message.r);
  my_vector->set_heading(message.heading);
  my_vector->set_speed(message.speed);
}

}  // namespace automotive
}  // namespace drake

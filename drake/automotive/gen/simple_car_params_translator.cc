#include "drake/automotive/gen/simple_car_params_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
SimpleCarParamsTranslator::AllocateOutputVector() const {
  return std::make_unique<SimpleCarParams<double>>();
}

void SimpleCarParamsTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const SimpleCarParams<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_simple_car_params_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.wheelbase = vector->wheelbase();
  message.track = vector->track();
  message.max_abs_steering_angle = vector->max_abs_steering_angle();
  message.max_velocity = vector->max_velocity();
  message.max_acceleration = vector->max_acceleration();
  message.velocity_limit_kp = vector->velocity_limit_kp();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void SimpleCarParamsTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<SimpleCarParams<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_simple_car_params_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message simple_car_params.");
  }
  my_vector->set_wheelbase(message.wheelbase);
  my_vector->set_track(message.track);
  my_vector->set_max_abs_steering_angle(message.max_abs_steering_angle);
  my_vector->set_max_velocity(message.max_velocity);
  my_vector->set_max_acceleration(message.max_acceleration);
  my_vector->set_velocity_limit_kp(message.velocity_limit_kp);
}

}  // namespace automotive
}  // namespace drake

#include "drake/automotive/gen/trajectory_car_params_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
TrajectoryCarParamsTranslator::AllocateOutputVector() const {
  return std::make_unique<TrajectoryCarParams<double>>();
}

void TrajectoryCarParamsTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const TrajectoryCarParams<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_trajectory_car_params_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.max_velocity = vector->max_velocity();
  message.velocity_limit_kp = vector->velocity_limit_kp();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void TrajectoryCarParamsTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector =
      dynamic_cast<TrajectoryCarParams<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_trajectory_car_params_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message trajectory_car_params.");
  }
  my_vector->set_max_velocity(message.max_velocity);
  my_vector->set_velocity_limit_kp(message.velocity_limit_kp);
}

}  // namespace automotive
}  // namespace drake

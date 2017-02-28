#include "drake/automotive/gen/bicycle_car_state_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
BicycleCarStateTranslator::AllocateOutputVector() const {
  return std::make_unique<BicycleCarState<double>>();
}

void BicycleCarStateTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const BicycleCarState<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_bicycle_car_state_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.Psi = vector->Psi();
  message.Psi_dot = vector->Psi_dot();
  message.beta = vector->beta();
  message.vel = vector->vel();
  message.sx = vector->sx();
  message.sy = vector->sy();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void BicycleCarStateTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<BicycleCarState<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_bicycle_car_state_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message bicycle_car_state.");
  }
  my_vector->set_Psi(message.Psi);
  my_vector->set_Psi_dot(message.Psi_dot);
  my_vector->set_beta(message.beta);
  my_vector->set_vel(message.vel);
  my_vector->set_sx(message.sx);
  my_vector->set_sy(message.sy);
}

}  // namespace automotive
}  // namespace drake

#include "drake/automotive/gen/endless_road_oracle_output_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
EndlessRoadOracleOutputTranslator::AllocateOutputVector() const {
  return std::make_unique<EndlessRoadOracleOutput<double>>();
}

void EndlessRoadOracleOutputTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const EndlessRoadOracleOutput<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_endless_road_oracle_output_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.net_delta_sigma = vector->net_delta_sigma();
  message.delta_sigma_dot = vector->delta_sigma_dot();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void EndlessRoadOracleOutputTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector =
      dynamic_cast<EndlessRoadOracleOutput<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_endless_road_oracle_output_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message endless_road_oracle_output.");
  }
  my_vector->set_net_delta_sigma(message.net_delta_sigma);
  my_vector->set_delta_sigma_dot(message.delta_sigma_dot);
}

}  // namespace automotive
}  // namespace drake

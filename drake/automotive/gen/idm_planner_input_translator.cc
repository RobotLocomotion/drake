#include "drake/automotive/gen/idm_planner_input_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
IdmPlannerInputTranslator::AllocateOutputVector() const {
  return std::make_unique<IdmPlannerInput<double>>();
}

void IdmPlannerInputTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const IdmPlannerInput<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_idm_planner_input_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.x = vector->x();
  message.v = vector->v();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void IdmPlannerInputTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<IdmPlannerInput<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_idm_planner_input_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message idm_planner_input.");
  }
  my_vector->set_x(message.x);
  my_vector->set_v(message.v);
}

}  // namespace automotive
}  // namespace drake

#include "drake/automotive/gen/bicycle_parameters_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
BicycleParametersTranslator::AllocateOutputVector() const {
  return std::make_unique<BicycleParameters<double>>();
}

void BicycleParametersTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const BicycleParameters<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_bicycle_parameters_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.mass = vector->mass();
  message.lf = vector->lf();
  message.lr = vector->lr();
  message.Iz = vector->Iz();
  message.Cf = vector->Cf();
  message.Cr = vector->Cr();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void BicycleParametersTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<BicycleParameters<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_bicycle_parameters_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error(
        "Failed to decode LCM message bicycle_parameters.");
  }
  my_vector->set_mass(message.mass);
  my_vector->set_lf(message.lf);
  my_vector->set_lr(message.lr);
  my_vector->set_Iz(message.Iz);
  my_vector->set_Cf(message.Cf);
  my_vector->set_Cr(message.Cr);
}

}  // namespace automotive
}  // namespace drake

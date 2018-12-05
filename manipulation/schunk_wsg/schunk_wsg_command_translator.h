#pragma once

#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

class SchunkWsgCommandTranslator : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgCommandTranslator)

  SchunkWsgCommandTranslator() : LcmAndVectorBaseTranslator(3) {}

  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
      systems::VectorBase<double>* vector_base) const override;

  void Serialize(double time,
      const systems::VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

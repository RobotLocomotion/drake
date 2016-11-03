#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include "drake/automotive/gen/linear_car_state.h"
#include "drake/common/drake_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/lcmt_linear_car_state_t.hpp"

namespace drake{
namespace automotive{

/**
 * Translates between LCM message objects and VectorBase objects for the
 * LinearCarState type.
 */
class DRAKE_EXPORT LinearCarStateTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  LinearCarStateTranslator()
      : LcmAndVectorBaseTranslator(LinearCarStateIndices::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
      systems::VectorBase<double>* vector_base) const override;
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake

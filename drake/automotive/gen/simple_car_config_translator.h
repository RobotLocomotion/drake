#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <vector>

#include "drake/automotive/gen/simple_car_config.h"
#include "drake/common/drake_export.h"
#include "drake/lcmt_simple_car_config_t.hpp"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * SimpleCarConfig type.
 */
class DRAKE_EXPORT SimpleCarConfigTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  SimpleCarConfigTranslator()
      : LcmAndVectorBaseTranslator(SimpleCarConfigIndices::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   systems::VectorBase<double>* vector_base) const override;
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake

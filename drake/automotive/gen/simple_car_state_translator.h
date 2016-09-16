#pragma once

// This file is generated by a script.  Do not edit!
// See drake/automotive/lcm_vector_gen.py.

#include "drake/automotive/gen/simple_car_state.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "lcmtypes/drake/lcmt_simple_car_state_t.hpp"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * SimpleCarState type.
 */
class DRAKEAUTOMOTIVE_EXPORT SimpleCarStateTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  SimpleCarStateTranslator()
      : LcmAndVectorBaseTranslator(SimpleCarStateIndices::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void TranslateLcmToVectorBase(
      const void* lcm_message_bytes, int lcm_message_length,
      systems::VectorBase<double>* vector_base) const override;
  void TranslateVectorBaseToLcm(
      double time, const systems::VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake

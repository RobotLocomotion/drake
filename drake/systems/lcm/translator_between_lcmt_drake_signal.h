#pragma once

#include <cstdint>
#include <vector>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes LcmAndVectorBaseTranslator to handle LCM messages of type
 * `drake::lcmt_drake_signal`.
 *
 * Assumes the number and order of values in the LCM message and the
 * drake::systems::VectorBase are identical.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorBetweenLcmtDrakeSignal
    : public LcmAndVectorBaseTranslator {
 public:
  /**
   * A constructor that stores the expected sizes of the LCM message and
   * the BasicVector. Both the LCM message and BasicVector are expected to have
   * the same size.
   *
   * @param[in] size The number of elements in both the
   * drake::systems::VectorBase and the `drake::lcmt_drake_signal`.
   */
  explicit TranslatorBetweenLcmtDrakeSignal(int size)
      : LcmAndVectorBaseTranslator(size) {}

  void TranslateLcmToVectorBase(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const override;

  void TranslateVectorBaseToLcm(
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

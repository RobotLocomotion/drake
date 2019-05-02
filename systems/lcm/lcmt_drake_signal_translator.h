#pragma once

#include <cstdint>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

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
class DRAKE_DEPRECATED("2019-08-01",
    "LcmtDrakeSignalTranslator is deprecated, with no replacement.")
LcmtDrakeSignalTranslator : public LcmAndVectorBaseTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmtDrakeSignalTranslator)

  /**
   * A constructor that sets the expected sizes of the LCM message and the
   * VectorBase. Both the LCM message and VectorBase must be the same size.
   *
   * @param[in] size The number of elements in both `VectorBase` and
   * `drake::lcmt_drake_signal`.
   */
  DRAKE_DEPRECATED("2019-08-01",
      "LcmtDrakeSignalTranslator is deprecated, with no replacement.")
  explicit LcmtDrakeSignalTranslator(int size)
      : LcmAndVectorBaseTranslator(size) {}

  DRAKE_DEPRECATED("2019-08-01",
      "LcmtDrakeSignalTranslator is deprecated, with no replacement.")
  void Deserialize(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const override;

  DRAKE_DEPRECATED("2019-08-01",
      "LcmtDrakeSignalTranslator is deprecated, with no replacement.")
  void Serialize(double time,
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

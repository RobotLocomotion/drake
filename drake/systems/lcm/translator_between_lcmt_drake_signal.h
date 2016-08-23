#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes `LcmAndVectorBaseTranslator` to handle LCM messages of type
 * `drake::lcmt_drake_signal`.
 *
 * Assumes that the number of values and the order of the values in the LCM
 * message and the `drake::systems::VectorBase` are identical.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorBetweenLcmtDrakeSignal
    : public LcmAndVectorBaseTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The number of elements in the
   * `drake::systems::VectorBase`. It is assumed that the
   * `drake::lcmt_drake_signal` LCM message has the same number of elements.
   */
  explicit TranslatorBetweenLcmtDrakeSignal(int size)
      : LcmAndVectorBaseTranslator(size) {}

  void TranslateLcmToVectorBase(
      const ::lcm::ReceiveBuffer* rbuf,
      VectorBase<double>* vector_base) const override;

  void PublishVectorBaseToLCM(
      const VectorBase<double>& vector_base,
          const std::string& channel, ::lcm::LCM* lcm) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

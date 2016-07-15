#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_basic_vector_translator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes the LcmBasicVectorTranslator to handle the LCM messages of type
 * `drake::lcmt_drake_signal`.
 *
 * Assumes that the order of the values in the LCM Vector and the BasicVector
 * are identical.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorLcmtDrakeSignal :
    public LcmBasicVectorTranslator {
 public:

  /**
   * The constructor.
   *
   * @param[in] size The number of elements in both the Basic Vector and the
   * LCM drake signal message.
   */
  explicit TranslatorLcmtDrakeSignal(int size) :
      LcmBasicVectorTranslator(size) {
  }

  void TranslateLcmToBasicVector(const ::lcm::ReceiveBuffer* rbuf,
      drake::systems::BasicVector<double>* basic_vector) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

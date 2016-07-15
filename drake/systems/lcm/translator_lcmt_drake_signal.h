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
class DRAKELCMSYSTEM2_EXPORT TranslatorLcmtDrakeSignal
    : public LcmBasicVectorTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The number of elements in both the Basic Vector and the
   * LCM drake signal message.
   */
  explicit TranslatorLcmtDrakeSignal(int size)
      : LcmBasicVectorTranslator(size) {}

  int get_message_data_length() const override;

  void TranslateLcmToBasicVector(
      const ::lcm::ReceiveBuffer* rbuf,
      BasicVector<double>* basic_vector) const override;

  void TranslateBasicVectorToLCM(
      const BasicVector<double>& basic_vector, uint8_t* const* data,
      int const* data_length) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

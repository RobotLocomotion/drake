#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_vector_interface_translator.h"
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
    : public LcmVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The number of elements in both the Basic Vector and the
   * LCM drake signal message.
   */
  explicit TranslatorLcmtDrakeSignal(int size)
      : LcmVectorInterfaceTranslator(size) {}

  // int get_message_data_length() const override;

  void TranslateLcmToVectorInterface(
      const ::lcm::ReceiveBuffer* rbuf,
      VectorInterface<double>* vector_interface) const override;

  void TranslateAndSendVectorInterfaceToLCM(
      const VectorInterface<double>& vector_interface,
          const std::string& channel, ::lcm::LCM* lcm) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

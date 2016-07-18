#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_vector_interface_translator.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes `LcmVectorInterfaceTranslator` to handle LCM messages of type
 * `drake::lcmt_drake_signal`.
 *
 * Assumes that the order of the values in the LCM message and the
 * `VectorInterface` are identical.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorLcmtDrakeSignal
    : public LcmVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The number of elements in both the `VectorInterface` and
   * the LCM drake signal message.
   */
  explicit TranslatorLcmtDrakeSignal(int size)
      : LcmVectorInterfaceTranslator(size) {}

  void TranslateLcmToVectorInterface(
      const ::lcm::ReceiveBuffer* rbuf,
      VectorInterface<double>* vector_interface) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

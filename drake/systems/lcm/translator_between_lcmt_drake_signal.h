#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_and_vector_interface_translator.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes `LcmAndVectorInterfaceTranslator` to handle LCM messages of type
 * `drake::lcmt_drake_signal`.
 *
 * Assumes that the number of values and the order of the values in the LCM
 * message and the `drake::systems::VectorInterface` are identical.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorBetweenLcmtDrakeSignal
    : public LcmAndVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The number of elements in the
   * `drake::systems::VectorInterface`. It is assumed that the
   * `drake::lcmt_drake_signal` LCM message has the same number of elements.
   */
  explicit TranslatorBetweenLcmtDrakeSignal(int size)
      : LcmAndVectorInterfaceTranslator(size) {}

  void TranslateLcmToVectorInterface(
      const ::lcm::ReceiveBuffer* rbuf,
      VectorInterface<double>* vector_interface) const override;

  void PublishVectorInterfaceToLCM(
      const VectorInterface<double>& vector_interface,
          const std::string& channel, ::lcm::LCM* lcm) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

#pragma once

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

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
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorLcmtDrakeSignal :
    public LcmBasicVectorTranslator {
 public:
  explicit TranslatorLcmtDrakeSignal(int size) :
      LcmBasicVectorTranslator(size) {
  }

  void TranslateLcmToBasicVector(const ::lcm::ReceiveBuffer* rbuf,
      drake::systems::BasicVector<double>* basic_vector) const override;

  // void TranslateBasicVectorToLCM(const BasicVector& basic_vector,
  //   const void *data, unsigned int datalen) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namesapce drake

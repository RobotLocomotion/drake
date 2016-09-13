#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"

#include <cstdint>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

using std::runtime_error;

void TranslatorBetweenLcmtDrakeSignal::TranslateLcmToVectorBase(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base);

  // Decodes the LCM message using data from the receive buffer.
  drake::lcmt_drake_signal message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw runtime_error(
      "drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal: "
          "TranslateLcmToBasicVector: ERROR: Failed to decode LCM message, the "
              "status is " + std::to_string(status) + ".");
  }

  // Verifies that the size of the LCM message matches the size of the basic
  // vector. Throws an exception if the sizes do not match.
  if (message.dim != vector_base->size()) {
    throw runtime_error(
      "drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal: "
      "TranslateLcmToBasicVector: ERROR: Size of LCM message (" +
      std::to_string(message.dim) +
      ") is not equal to the size of the vector vector (" +
      std::to_string(vector_base->size()) + ").");
  }

  // Saves the values in from the LCM message into the basic vector.
  // Assumes that the order of the values in both vectors are identical.
  for (int ii = 0; ii < message.dim; ++ii) {
    vector_base->SetAtIndex(ii, message.val[ii]);
  }
}

void TranslatorBetweenLcmtDrakeSignal::TranslateVectorBaseToLcm(
    const VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  DRAKE_ASSERT(vector_base.size() == get_vector_size());
  DRAKE_ASSERT(lcm_message_bytes != nullptr);

  // Instantiates and initializes a LCM message capturing the state of
  // parameter vector_base.
  drake::lcmt_drake_signal message;
  message.dim = vector_base.size();
  message.val.resize(message.dim);
  message.coord.resize(message.dim);

  for (int ii = 0; ii < message.dim; ++ii) {
    message.val[ii] = vector_base.GetAtIndex(ii);
  }

  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

#include "drake/systems/lcm/translator_lcmt_drake_signal.h"

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

using std::runtime_error;

void TranslatorLcmtDrakeSignal::TranslateLcmToBasicVector(
    const ::lcm::ReceiveBuffer* rbuf,
    drake::systems::BasicVector<double>* basic_vector) const {
  // Checks if basic_vector is nullptr. Throws an exception if it is.
  if (basic_vector == nullptr) {
    throw runtime_error(
      "drake::systems::lcm::TranslatorLcmtDrakeSignal: "
      "TranslateLcmToBasicVector: ERROR: Parameter basic_vector is null.");
  }

  // Decodes the LCM message using data from the receive buffer.
  drake::lcmt_drake_signal message;
  int status = message.decode(rbuf->data, 0, rbuf->data_size);
  if (status < 0) {
    throw runtime_error(
      "drake::systems::lcm::TranslatorLcmtDrakeSignal: "
      "TranslateLcmToBasicVector: ERROR: Failed to decode LCM message.");
  }

  // Verifies that the size of the LCM message matches the size of the basic
  // vector. Throws an exception if the sizes do not match.
  if (message.dim != basic_vector->size()) {
    throw runtime_error("drake::systems::lcm::TranslatorLcmtDrakeSignal: "
      "TranslateLcmToBasicVector: ERROR: Size of LCM message (" +
      std::to_string(message.dim) +
      ") is not equal to the size of the basic vector (" +
      std::to_string(basic_vector->size()) + ").");
  }
  Eigen::VectorBlock<VectorX<double>> basic_vector_value =
    basic_vector->get_mutable_value();

  // Saves the values in from the LCM message into the basic vector.
  // Assumes that the order of the values in both vectors are identical.
  for (int ii = 0; ii < message.dim; ++ii) {
    basic_vector_value[ii] = message.val[ii];
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

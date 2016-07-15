#include "drake/systems/lcm/translator_lcmt_drake_signal.h"

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

using std::runtime_error;

int TranslatorLcmtDrakeSignal::get_message_data_length() const {
  drake::lcmt_drake_signal message;
  message.dim = get_basic_vector_size();
  message.val.resize(message.dim);
  message.coord.resize(message.dim);
  unsigned int data_length = message.getEncodedSize();
  return static_cast<int>(data_length);
}

void TranslatorLcmtDrakeSignal::TranslateLcmToBasicVector(
    const ::lcm::ReceiveBuffer* rbuf,
    drake::systems::BasicVector<double>* basic_vector) const {
  DRAKE_ABORT_UNLESS(basic_vector);

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

void TranslatorLcmtDrakeSignal::TranslateBasicVectorToLCM(
    const BasicVector<double>& basic_vector, uint8_t* const* data,
    int const* datalen) const {

  // TODO(liang.fok) Assert that basic_vector.size == get_basic_vector_size()

  // Instantiates and initializes a LCM message containing the information
  // contained within parameter basic_vector.
  drake::lcmt_drake_signal message;
  message.dim = basic_vector.size();
  message.val.resize(message.dim);
  message.coord.resize(message.dim);

  Eigen::VectorBlock<const VectorX<double>> values = basic_vector.get_value();

  for (int ii = 0; ii < message.dim; ++ii) {
    message.val[ii] = values[ii];
    message.coord[ii] = "Coord_" + std::to_string(ii);
  }

  message.encode(*data, 0, *datalen);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

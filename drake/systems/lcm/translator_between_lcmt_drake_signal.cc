#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

using std::runtime_error;

void TranslatorBetweenLcmtDrakeSignal::TranslateLcmToVectorInterface(
    const ::lcm::ReceiveBuffer* rbuf,
    VectorInterface<double>* vector_interface) const {
  DRAKE_ABORT_UNLESS(vector_interface);

  // Decodes the LCM message using data from the receive buffer.
  drake::lcmt_drake_signal message;
  int status = message.decode(rbuf->data, 0, rbuf->data_size);
  if (status < 0) {
    throw runtime_error(
      "drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal: "
          "TranslateLcmToBasicVector: ERROR: Failed to decode LCM message, the "
              "status is " + std::to_string(status) + ".");
  }

  // Verifies that the size of the LCM message matches the size of the basic
  // vector. Throws an exception if the sizes do not match.
  if (message.dim != vector_interface->size()) {
    throw runtime_error(
      "drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal: "
      "TranslateLcmToBasicVector: ERROR: Size of LCM message (" +
      std::to_string(message.dim) +
      ") is not equal to the size of the vector vector (" +
      std::to_string(vector_interface->size()) + ").");
  }
  Eigen::VectorBlock<VectorX<double>> vector_interface_value =
    vector_interface->get_mutable_value();

  // Saves the values in from the LCM message into the basic vector.
  // Assumes that the order of the values in both vectors are identical.
  for (int ii = 0; ii < message.dim; ++ii) {
    vector_interface_value[ii] = message.val[ii];
  }
}

void TranslatorBetweenLcmtDrakeSignal::PublishVectorInterfaceToLCM(
    const VectorInterface<double>& vector_interface, const std::string& channel,
    ::lcm::LCM* lcm) const {

  DRAKE_ASSERT(vector_interface.size() == get_vector_size());

  // Instantiates and initializes a LCM message capturing the state of
  // parameter vector_interface.
  drake::lcmt_drake_signal message;
  message.dim = vector_interface.size();
  message.val.resize(message.dim);
  message.coord.resize(message.dim);

  Eigen::VectorBlock<const VectorX<double>> values =
      vector_interface.get_value();

  for (int ii = 0; ii < message.dim; ++ii) {
    message.val[ii] = values[ii];
  }

  lcm->publish(channel, &message);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

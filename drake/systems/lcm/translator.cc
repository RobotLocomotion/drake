#include "drake/systems/lcm/translator.h"

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

void MyLcmtDrakeSignalTranslator::InitializeMessage(
    lcmt_drake_signal* msg) const {
  msg->dim = 0;
  msg->val.resize(msg->dim, 0);
  msg->coord.resize(msg->dim);
  msg->timestamp = 0;
}

void MyLcmtDrakeSignalTranslator::Decode(
    const lcmt_drake_signal& msg, double* time,
    VectorBase<double>* vector_base) const {
  if (msg.dim != vector_base->size()) {
    throw std::runtime_error(
        "drake::systems::lcm::LcmtDrakeSignalTranslator: Decode: ERROR: "
        "The LCM message's size (" +
        std::to_string(msg.dim) +
        ") is not "
        "equal to vector_base's size (" +
        std::to_string(vector_base->size()) + ").");
  }

  // Saves the values from the LCM message into vector_base.
  // Assumes that the order of the values are identical in both.
  for (int i = 0; i < msg.dim; ++i) {
    vector_base->SetAtIndex(i, msg.val[i]);
  }

  *time = static_cast<double>(msg.timestamp) / 1e3;
}

void MyLcmtDrakeSignalTranslator::Encode(double time,
                                         const VectorBase<double>& vector_base,
                                         lcmt_drake_signal* msg) const {
  msg->dim = vector_base.size();
  msg->val.resize(msg->dim);
  msg->coord.resize(msg->dim);
  msg->timestamp = static_cast<int64_t>(time * 1e3);

  for (int i = 0; i < msg->dim; ++i) {
    msg->val[i] = vector_base.GetAtIndex(i);
    msg->coord[i] = "";
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

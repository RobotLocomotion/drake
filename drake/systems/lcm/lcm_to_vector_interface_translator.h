#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert from
 * LCM message objects to `drake::systems::VectorInterface` objects.
 */
class LcmToVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the vector in the `VectorInterface`.
   */
  explicit LcmToVectorInterfaceTranslator(int size) : size_(size) {
  }

  // Disable copy and assign.
  LcmToVectorInterfaceTranslator(const LcmToVectorInterfaceTranslator&) =
      delete;
  LcmToVectorInterfaceTranslator& operator=(
      const LcmToVectorInterfaceTranslator&) = delete;

  /**
   * Returns the size of the vector in the `VectorInterface`.
   */
  int get_vector_size() const {
    return size_;
  }

  /**
   * Translates an LCM message into a `VectorInterface` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] vector_interface A pointer to where the translation of the LCM
   * message should be stored. This pointer must not be `nullptr` and must
   * be valid throughout the duration of this method call.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message's size does not equal the size of
   * @p vector_interface.
   */
  virtual void TranslateLcmToVectorInterface(const ::lcm::ReceiveBuffer* rbuf,
    VectorInterface<double>* vector_interface) const = 0;

 private:
  // The size of the vector in the VectorInterface.
  const int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

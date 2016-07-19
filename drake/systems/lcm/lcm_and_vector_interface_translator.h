#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert between
 * LCM message objects and `drake::systems::VectorInterface` objects.
 */
class LcmAndVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the vector in the `VectorInterface`.
   */
  explicit LcmAndVectorInterfaceTranslator(int size) : size_(size) {
  }

  // Disable copy and assign.
  LcmAndVectorInterfaceTranslator(const LcmAndVectorInterfaceTranslator&) =
      delete;
  LcmAndVectorInterfaceTranslator& operator=(
      const LcmAndVectorInterfaceTranslator&) = delete;

  /**
   * Returns the size of the vector in the `drake::systems::VectorInterface`
   * object.
   */
  int get_vector_size() const {
    return size_;
  }

  /**
   * Translates an LCM message into a `drake::systems::VectorInterface` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] vector_interface A pointer to where the translation of the LCM
   * message should be stored. This pointer must not be `nullptr` and must
   * be valid throughout the duration of this method call.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message is incompatible with the @p vector_interface.
   * This often occurs when the size of the @p vector_interface does not equal
   * or is incompatible with the size of the decoded LCM message.
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

#pragma once

#include <cstdint>
#include <vector>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert between
 * LCM message bytes and `drake::systems::VectorBase` objects.
 */
class LcmAndVectorBaseTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the vector in the `VectorBase`.
   */
  explicit LcmAndVectorBaseTranslator(int size) : size_(size) {
  }

  // Disable copy and assign.
  LcmAndVectorBaseTranslator(const LcmAndVectorBaseTranslator&) =
      delete;
  LcmAndVectorBaseTranslator& operator=(
      const LcmAndVectorBaseTranslator&) = delete;

  /**
   * Returns the size of the vector in the `drake::systems::VectorBase`
   * object.
   */
  int get_vector_size() const {
    return size_;
  }

  /**
   * Translates LCM message bytes into a `drake::systems::VectorBase` object.
   *
   * @param[in] lcm_message_bytes A pointer to a buffer holding the LCM
   * message's data.
   *
   * @param[in] lcm_message_length The number of bytes pointed to by
   * the @p lcm_message_bytes.
   *
   * @param[out] vector_base A pointer to where the translation of the LCM
   * message should be stored. This pointer must not be `nullptr`.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message is incompatible with the @p vector_base.
   * This often occurs when the size of the @p vector_base does not equal
   * or is incompatible with the size of the decoded LCM message.
   */
  virtual void TranslateLcmToVectorBase(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const = 0;

  /**
   * Translates a `drake::systems::VectorBase` object into LCM message bytes.
   *
   * @param[in] vector_base The object to convert into an LCM message.
   *
   * @param[out] lcm_message_bytes The LCM message bytes.
   * This pointer must not be `nullptr`.
   */
  virtual void TranslateVectorBaseToLcm(
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const = 0;

 private:
  // The size of the vector in the VectorBase.
  const int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

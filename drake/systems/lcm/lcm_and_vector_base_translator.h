#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert between
 * LCM message objects and `drake::systems::VectorBase` objects.
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
   * Translates an LCM message into a `drake::systems::VectorBase` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] vector_base A pointer to where the translation of the LCM
   * message should be stored. This pointer must not be `nullptr`.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message is incompatible with the @p vector_base.
   * This often occurs when the size of the @p vector_base does not equal
   * or is incompatible with the size of the decoded LCM message.
   */
  virtual void TranslateLcmToVectorBase(const ::lcm::ReceiveBuffer* rbuf,
    VectorBase<double>* vector_base) const = 0;

  /**
   * Translates a `drake::systems::VectorBase` object into an LCM message
   * and publishes it on the specified LCM channel.
   *
   * @param[in] vector_base A reference to the object to convert into an
   * LCM message.
   *
   * @param[in] channel The name of the channel on which to publish the LCM
   * message.
   *
   * @param[in] lcm A pointer to the LCM subsystem. This pointer must not be
   * `nullptr`.
   */
  virtual void PublishVectorBaseToLCM(
      const VectorBase<double>& vector_base,
      const std::string& channel, ::lcm::LCM* lcm) const = 0;

 private:
  // The size of the vector in the VectorBase.
  const int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

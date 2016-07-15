#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert between
 * LCM message objects and drake::systems::BasicVector objects.
 */
class LcmVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the basic vector.
   */
  explicit LcmVectorInterfaceTranslator(int size) : size_(size) {
  }

  // Disable copy and assign.
  LcmVectorInterfaceTranslator(const LcmVectorInterfaceTranslator&) = delete;
  LcmVectorInterfaceTranslator& operator=(const LcmVectorInterfaceTranslator&)
    = delete;

  /**
   * Returns the size of the basic vector and the vector representation of the
   * LCM message.
   */
  int get_vector_size() const {
    return size_;
  }

  /**
   * Returns the size of the LCM message in bytes.
   */
  virtual int get_message_data_length() const = 0;

  /**
   * Translates an LCM message into a `BasicVector` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] vector_interface A pointer to where the translation of the LCM
   * message should be stored. This pointer must not be nullptr and must remain
   * valid throughout the duration of this method call.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message's size does not equal the size of
   * @p basic_vector.
   */
  virtual void TranslateLcmToVectorInterface(const ::lcm::ReceiveBuffer* rbuf,
    VectorInterface<double>* vector_interface) const = 0;

  /**
   * Translates a `BasicVector` object into an LCM message.
   *
   * @param[in] basic_vector The basic vector to convert into an LCM message.
   *
   * @param[in] data A pointer to where the encoded data should be saved.
   *
   * @param[in] data_length The number of bytes of encoded data. This is the
   * amount of data being pointed to by @p data.
   */
  virtual void TranslateVectorInterfaceToLCM(
      const VectorInterface<double>& basic_vector, uint8_t* const* data,
      int const* data_length) const = 0;

 private:
  // The size of the basic vector and vector representation of the LCM message.
  const int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

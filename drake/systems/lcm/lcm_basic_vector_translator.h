#pragma once

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert between
 * LCM message objects and drake::systems::BasicVector objects.
 */
class LcmBasicVectorTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the basic vector.
   */
  explicit LcmBasicVectorTranslator(int size) : size_(size) {
  }

  /**
   * Returns the size of the basic vector and the vector representation of the
   * LCM message.
   */
  int get_basic_vector_size() const {
    return size_;
  }

  /**
   * Translates an LCM message a `BasicVector` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] basic_vector A pointer to where the translation of the LCM
   * message should be stored. A runtime exception is thrown if this parameter
   * is nullptr.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message's size does not equal the size of
   * @p basic_vector.
   */
  virtual void TranslateLcmToBasicVector(const ::lcm::ReceiveBuffer* rbuf,
    drake::systems::BasicVector<double>* basic_vector) const = 0;

  // virtual void TranslateBasicVectorToLCM(const BasicVector& basic_vector,
  //   const void *data, unsigned int datalen) const = 0;

 private:
  // The size of the basic vector and vector representation of the LCM message.
  int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

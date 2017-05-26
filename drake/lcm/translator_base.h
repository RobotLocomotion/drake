#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace lcm {

/**
 * Base API for a translator between arbitrary data of DataType and a Lcm
 * message of MsgType.
 */
template <typename DataType, typename MsgType>
class TranslatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TranslatorBase)

  TranslatorBase() {}
  virtual ~TranslatorBase() {}

  /**
   * Returns a const reference to the default data value.
   */
  virtual const DataType& get_default_data() const = 0;

  /**
   * Returns a const reference to the default message value.
   */
  virtual const MsgType& get_default_msg() const = 0;

  /**
   * Fills @p msg with contents from @p data and @p time. When @p data contains
   * information about time, it is up to the implementation to pick one for
   * encoding purposes.
   */
  virtual void Encode(
      double time, const DataType& data, MsgType* msg) const = 0;

  /**
   * Fills @p data and @p time with contents from @p msg.
   */
  virtual void Decode(
      const MsgType& msg, double* time, DataType* data) const = 0;
};

}  // namespace lcm
}  // namespace drake

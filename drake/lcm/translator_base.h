#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace lcm {

/**
 * Base API for a translator between arbitrary data of DataType and a Lcm
 * message of MsgType. This class is intended to facilitate Lcm communication
 * under the System framework. Within the System frame, time can be acquired
 * directly from the Context, so it is not uncommon to omit time from state
 * representations (DataType). Meanwhile, a well defined Lcm message (MsgType)
 * almost always contains an time stamp. Thus, during message encoding
 * (DataType -> MsgType), we provide an additional time information (often
 * extracted from some Context). During decoding (MsgType -> DataType), the
 * message's time stamp can be extracted and stored separated. This introduces
 * an asymmetry of information flow, i.e.
 * <pre>
 *   (DataType, time) -> MsgType,
 *   MsgType -> (DataType, time)
 * </pre>
 *
 * Note that MsgType is not required to be a Lcm message. In fact, it can be
 * any arbitrary data type as well.
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
  virtual void Encode(const DataType& data, MsgType* msg) const = 0;

  /**
   * Fills @p data and @p time with contents from @p msg.
   */
  virtual void Decode(const MsgType& msg, DataType* data) const = 0;

  virtual void EncodeTime(double, MsgType*) const {}

  virtual void DecodeTime(const MsgType&, double*) const {}
};

}  // namespace lcm
}  // namespace drake

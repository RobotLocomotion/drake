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
 * (DataType -> MsgType), we provide additional time information (often
 * extracted from some Context). During decoding (MsgType -> DataType), the
 * message's time stamp can be extracted and stored separately. This introduces
 * an asymmetry of information flow, i.e.
 * <pre>
 *   DataType and time -> MsgType,
 *   MsgType -> DataType and time
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
   * Encodes @p data into @p msg.
   */
  virtual void Encode(const DataType& data, MsgType* msg) const = 0;

  /**
   * Decodes @p msg into @p data.
   */
  virtual void Decode(const MsgType& msg, DataType* data) const = 0;

  /**
   * Encodes @p time into @p msg. The default implementation does nothing.
   */
  virtual void EncodeTime(double time, MsgType* msg) const {}

  /**
   * Decodes @p msg into @p time. The default implementation does nothing.
   */
  virtual void DecodeTime(const MsgType& msg, double* time) const {}
};

}  // namespace lcm
}  // namespace drake

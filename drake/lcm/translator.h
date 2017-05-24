#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace lcm {

template <typename DataType, typename MsgType>
class TranslatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TranslatorBase)

  TranslatorBase() {}
  virtual ~TranslatorBase() {}

  virtual std::unique_ptr<DataType> AllocateDecodedData() const = 0;
  virtual void InitializeMessage(MsgType* msg) const = 0;

  virtual void Encode(double time, const DataType& data, MsgType* msg) const = 0;
  virtual void Decode(const MsgType& msg,  double* time, DataType* data) const = 0;
};

}  // namespace lcm
}  // namespace drake

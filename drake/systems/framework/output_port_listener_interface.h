#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace detail {

/// OutputPortListenerInterface is an interface that consumers of an output
/// port must satisfy to receive notifications when the value on that output
/// port's version number is incremented.
class OutputPortListenerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputPortListenerInterface)

  OutputPortListenerInterface();
  virtual ~OutputPortListenerInterface();

  /// Invalidates any data that depends on the OutputPort. Called whenever
  /// the OutputPort's version number is incremented.
  virtual void Invalidate() = 0;

  /// Notifies the consumer that the OutputPort is no longer valid and should
  /// not be read.
  virtual void Disconnect() = 0;
};

}  // namespace detail
}  // namespace systems
}  // namespace drake

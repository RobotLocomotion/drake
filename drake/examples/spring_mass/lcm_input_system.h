#pragma once

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename ScalarType, typename LCMMessageType>
class LCMInputSystem<ScalarType> :
    public LCMVector<ScalarType, > {
 public:
  virtual ~LCMVector() {}

  /// Encodes an LCM message as an LCMVector.
  virtual void Encode(const double& t, const LCMMessageType& message) = 0;

  /// Decodes an LCMVector into an LCM message.
  virtual void Decode() = 0;

 protected:
  LCMVector() {}
};

}  // namespace systems
}  // namesapce drake

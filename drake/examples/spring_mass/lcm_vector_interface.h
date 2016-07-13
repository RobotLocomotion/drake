#pragma once

#include "drake/systems/framework/vector_interface.h"

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

namespace drake {
namespace systems {

/// This abstract class is a specialization of `drake::systems::VectorInterface`
/// that adds support for conversion to and from LCM messages.
template <typename ScalarType, typename LCMMessageType>
class LCMVectorInterface :
    public VectorInterface<ScalarType> {
 public:
  virtual ~LCMVectorInterface() {}

  /// Converts a LCM message into a `LCMVectorInterface`.
  ///
  /// @param[in] message The LCM message containing the state to save within
  /// this `LCMVectorInterface`.
  virtual void Encode(const LCMMessageType& message) = 0;

  /// Converts this `LCMVectorInterface` into a LCM message.
  ///
  /// @param[out] message A pointer to the LCM message in which to save this
  /// this `LCMVectorInterface`'s state.
  virtual void Decode(LCMMessageType* message) = 0;

 protected:
  LCMVectorInterface() {}
};

}  // namespace systems
}  // namesapce drake

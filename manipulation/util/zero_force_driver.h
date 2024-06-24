#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

namespace drake {
namespace manipulation {

/** A driver that applies zero actuation to every joint of a model.  Useful for
debugging and testing; useless in reality.  No LCM channels are created. */
struct ZeroForceDriver {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ZeroForceDriver);
  ZeroForceDriver() = default;

  // Currently there is no configuration here.
  template <typename Archive>
  void Serialize(Archive*) {}
};

}  // namespace manipulation
}  // namespace drake

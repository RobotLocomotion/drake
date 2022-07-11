#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

namespace drake {
namespace lcm {

/** The set of parameters for configuring DrakeLcm.  */
struct DrakeLcmParams {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DrakeLcmParams)
  DrakeLcmParams() = default;
  ~DrakeLcmParams();

  /** The URL for DrakeLcm communication. If empty, DrakeLcm will use the
  default URL per the DrakeLcm::DrakeLcm() no-argument constructor. */
  std::string lcm_url;

  /** The custom LCM channel name suffix for this DrakeLcm instance (optional).

  When provided, calls to DrakeLcm::Publish() or DrakeLcm::Subscribe() will
  append this string to the `channel` name requested for publish or subscribe.

  The callback of DrakeLcm::SubscribeAllChannels() will receive the "base"
  channel name WITHOUT this suffix; messages received without the configured
  suffix will be discarded.
  */
  std::string channel_suffix;

  /** (Advanced) Controls whether or not LCM's background receive thread will
  be launched immediately during the constructor (when false) or deferred until
  the first time it's needed (when true). This can be useful if the scheduling
  configuration for new threads varies between the construction time and first
  use. */
  bool defer_initialization{false};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(lcm_url));
    a->Visit(DRAKE_NVP(channel_suffix));
    a->Visit(DRAKE_NVP(defer_initialization));
  }
};

}  // namespace lcm
}  // namespace drake

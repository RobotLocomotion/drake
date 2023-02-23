#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

namespace drake {
namespace lcm {

/** The set of parameters for configuring DrakeLcm.  */
struct DrakeLcmParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(lcm_url));
    a->Visit(DRAKE_NVP(channel_suffix));
    a->Visit(DRAKE_NVP(defer_initialization));
  }

  /** The URL for DrakeLcm communication. If empty, DrakeLcm will use the
  default URL per the DrakeLcm::DrakeLcm() no-argument constructor. */
  std::string lcm_url;

  /** The custom LCM channel name suffix for this DrakeLcm instance (optional).

  When provided, calls to DrakeLcm::Publish() or DrakeLcm::Subscribe() will
  append this string to the `channel` name requested for publish or subscribe.

  For example, with the channel_suffix set to "_ALT" a call to
  `Publish(&drake_lcm, "FOO", message)` will transmit on the network using the
  channel name "FOO_ALT", and a call to `Subscribe(&lcm, "BAR", handler)` will
  only call the handler for messages received on the "BAR_ALT" channel name.

  Simiarly, DrakeLcm::SubscribeMultichannel() and
  DrakeLcm::SubscribeAllChannels() only subscribe to network messages
  that end with the suffix. A network message on a non-matching channel name
  (e.g., "QUUX") will silently discarded.
  The DrakeLcmInterface::MultichannelHandlerFunction callback will be passed the
  _unadaorned_  channel name as its first argument (e.g., "FOO" or "BAR"), not
  "FOO_ALT", etc. */
  std::string channel_suffix;

  /** (Advanced) Controls whether or not LCM's background receive thread will
  be launched immediately during the constructor (when false) or deferred until
  the first time it's needed (when true). This can be useful if the scheduling
  configuration for new threads varies between the construction time and first
  use. */
  bool defer_initialization{false};
};

}  // namespace lcm
}  // namespace drake

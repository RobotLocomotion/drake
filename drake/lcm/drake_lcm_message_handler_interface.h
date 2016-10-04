#pragma once

#include <cstdint>

#include "drake/drakeLcm_export.h"

namespace drake {
namespace lcm {

/**
 * Defines the message handler interface to implemented by all of Drake's LCM
 * subscriptions. This avoids DrakeLcmInterface from needing to be templated.
 */
class DRAKELCM_EXPORT DrakeLcmMessageHandlerInterface {
 public:
  /**
   * This method is called when an LCM message arrives over the subscribed
   * channel.
   *
   * @param[in] channel The channel on which the LCM message was received.
   *
   * @param[in] message_buffer A pointer to the byte vector that is the serial
   * representation of the LCM message.
   *
   * @param[in] message_size The size of @p message_buffer.
   */
  virtual void HandleMessage(const std::string& channel,
      const void* message_buffer, int message_size) = 0;
};

}  // namespace lcm
}  // namespace drake

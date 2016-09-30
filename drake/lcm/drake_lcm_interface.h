#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A top-level abstract class that for enabling users to seamlessly switch
 * between _real_ and _mock_ LCM instances.
 */
class DRAKELCM_EXPORT DrakeLcmInterface {
 public:
  /**
   * Publishes a raw data LCM message.
   *
   * @param[in] channel the channel to publish the message on.
   * @param[in] data data buffer containing the message to publish
   * @param[in] datalen length of the message, in bytes.
   */
  virtual void Publish(const std::string& channel, const void *data,
                       unsigned int datalen) = 0;

  /**
   * Subscribes a callback method of an object to an LCM channel, without
   * automatic message decoding.
   *
   * This method is designed for use when automatic message decoding is
   * not desired.
   *
   * The callback method will be invoked on the object when a message
   * arrives on the specified channel.
   *
   * @param[in] channel The channel to subscribe to.
   *
   * @param[in] handlerMethod A class method pointer identifying the callback
   * method.
   *
   * @param handler A class instance that the callback method will be
   * invoked on.
   */
  virtual void Subscribe(const std::string& channel,
      void (DrakeLcmMessageHandlerInterface::*HandleMessage)(const uint8_t* message_buffer, uint32_t message_size),
      DrakeLcmMessageHandlerInterface* handler) = 0;
};

}  // namespace lcm
}  // namespace drake

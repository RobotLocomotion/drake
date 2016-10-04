#pragma once

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A top-level abstract class that enables LCM to be mocked.
 */
class DRAKELCM_EXPORT DrakeLcmInterface {
 public:
  /**
   * Starts the receive thread. This must be called for subscribers to receive
   * any messages.
   *
   * @pre StartReceiveThread() was not called.
   */
  virtual void StartReceiveThread() = 0;

  /**
   * Stops the receive thread. This must be called prior to any subscribers
   * being destroyed.
   *
   * @pre StartReceiveThread() was called.
   */
  virtual void StopReceiveThread() = 0;

  /**
   * Publishes a raw data LCM message.
   *
   * @param[in] channel The channel on which to publish the message.
   *
   * @param[in] data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param[in] data_size The length of @data in bytes.
   */
  virtual void Publish(const std::string& channel, const void *data,
                       int data_size) = 0;

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
   * @param[in] HandleMessage A class method pointer identifying the callback
   * method.
   *
   * @param handler A class instance whose callback method will be invoked.
   */
  virtual void Subscribe(const std::string& channel,
      void (DrakeLcmMessageHandlerInterface::*HandleMessage)(
          const std::string& channel, const void* message_buffer,
          int message_size),
      DrakeLcmMessageHandlerInterface* handler) = 0;
};

}  // namespace lcm
}  // namespace drake

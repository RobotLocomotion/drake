#pragma once

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A pure virtual interface that enables LCM to be mocked.
 */
class DRAKELCM_EXPORT DrakeLcmInterface {
 public:
  virtual ~DrakeLcmInterface() {}

  /**
   * Starts the receive thread. This must be called for subscribers to receive
   * any messages.
   *
   * @pre StartReceiveThread() was not called.
   */
  virtual void StartReceiveThread() = 0;

  /**
   * Stops the receive thread. This must be called prior to any subscribers
   * being destroyed. Note that the receive thread will be automatically stopped
   * by this class's destructor, so usage of this method will be extremely rare.
   * It will only be needed if this class's instance and the subscribers to LCM
   * channels are owned by different classes. In such a scenario, this method
   * can be used to ensure the receive thread is destroyed before the
   * subscribers are destroyed.
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
  virtual void Publish(const std::string& channel, const void* data,
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
   * @param[in] handler A class instance whose callback method will be invoked.
   */
  virtual void Subscribe(const std::string& channel,
      DrakeLcmMessageHandlerInterface* handler) = 0;
};

}  // namespace lcm
}  // namespace drake

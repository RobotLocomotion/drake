#pragma once

#include <limits>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A pure virtual interface that enables LCM to be mocked.
 */
class DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmInterface)
  DrakeLcmInterface() = default;
  virtual ~DrakeLcmInterface() = default;

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
   * Publishes an LCM message on channel @p channel.
   *
   * @param[in] channel The channel on which to publish the message.
   *
   * @param[in] data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param[in] data_size The length of @data in bytes.
   *
   * @param[in] time_sec Time in seconds when the publish event occurred. Note
   * that this argument is only used when generating a Lcm log.
   */
  virtual void Publish(const std::string& channel, const void* data,
                       int data_size, double time_sec = 0) = 0;

  /**
   * Subscribes to an LCM channel without automatic message decoding. The
   * callback method within @p handler will be invoked when a message arrives on
   * channel @p channel.
   *
   * @param[in] channel The channel to subscribe to.
   *
   * @param[in] handler A class instance whose callback method will be invoked.
   */
  virtual void Subscribe(const std::string& channel,
                         DrakeLcmMessageHandlerInterface* handler) = 0;

  /**
   * Only used for supporting Lcm log playback. Default implementation is no-op.
   * @see DrakeLcmLog::DispatchMessageAndAdvanceLog
   */
  virtual void DispatchMessageAndAdvanceLog(double) {}

  /**
   * Only used for supporting Lcm log playback. Default implementation returns
   * infinity.
   * @see DrakeLcmLog::GetNextMessageTime
   */
  virtual double GetNextMessageTime() const {
    return std::numeric_limits<double>::infinity();
  }
};

}  // namespace lcm
}  // namespace drake

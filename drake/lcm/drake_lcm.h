#pragma once

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace lcm {

// template <class MessageHandlerClass>
// class DrakeLcmSubscription : public Subscription {
//     friend class DrakeLcmInterface;
//     private:
//         MessageHandlerClass* handler;
//         void (MessageHandlerClass::*handlerMethod)(const uint8_t* message_buffer,
//           int message_size);
//         static void cb_func(const lcm_recv_buf_t *rbuf, const char *channel,
//                 void *user_data)
//         {
//             LCMMHSubscription<MessageType,MessageHandlerClass> *subs =
//                 static_cast<LCMMHSubscription<MessageType,MessageHandlerClass> *>(user_data);
//             MessageType msg;
//             int status = msg.decode(rbuf->data, 0, rbuf->data_size);
//             if (status < 0) {
//                 fprintf (stderr, "error %d decoding %s!!!\n", status,
//                         MessageType::getTypeName());
//                 return;
//             }
//             const ReceiveBuffer rb = {
//                 rbuf->data,
//                 rbuf->data_size,
//                 rbuf->recv_utime
//             };
//             std::string chan_str(channel);
//             (subs->handler->*subs->handlerMethod)(&rb, chan_str, &msg);
//         }
// };

/**
 * A wrapper around a *real* LCM instance.
 */
template <class MessageHandlerClass>
class DRAKELCM_EXPORT DrakeLcm : public DrakeLcmInterface<MessageHandlerClass> {
 public:

  /**
   * An accessor to the LCM instance encapsulated by this object. The returned
   * pointer is guaranteed to be valid for the duration of this object's
   * lifetime.
   */
  ::lcm::LCM* get_lcm_instance() {
    return &lcm_;
  }

  /**
   * A constructor that initializes the real LCM instance.
   */
  // explicit DrakeLcm() {}

  /**
   * Publishes an LCM message.
   *
   * @param[in] channel The LCM channel name.
   *
   * @param[in] data A pointer to an array of bytes that isthe serialized LCM
   * message to publish.
   *
   * @param[in] data_length The number of bytes in @p data.
   */
  void Publish(const std::string& channel, const void *data,
               unsigned int data_length) {
    lcm_.publish(channel, data, data_length);
  }

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
  void Subscribe(const std::string& channel,
      void (MessageHandlerClass::*handlerMethod)(
          const uint8_t* message_buffer, uint32_t message_length),
      MessageHandlerClass* handler) {

  }

 private:
  // The real LCM intance that's encapsulated by this class.
  ::lcm::LCM lcm_;
};

}  // namespace lcm
}  // namespace drake

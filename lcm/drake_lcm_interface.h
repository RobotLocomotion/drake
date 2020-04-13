#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace lcm {

// Declared later in this file.
class DrakeLcmInterface;
class DrakeSubscriptionInterface;

namespace internal {
// Used by the drake::lcm::Subscribe() free function to report errors.
void OnHandleSubscriptionsError(
    DrakeLcmInterface* lcm,
    const std::string& error_message);
}  // namespace internal

/**
 * A pure virtual interface that enables LCM to be mocked.
 *
 * Because it must be pure, in general it will receive breaking API changes
 * without notice.  Users should not subclass this interface directly, but
 * rather use one of the existing subclasses instead.
 *
 * Similarly, method arguments will receive breaking API changes without
 * notice.  Users should not call this interface directly, but rather use
 * drake::lcm::Publish() or drake::lcm::Subscribe() instead.
 */
class DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmInterface)
  virtual ~DrakeLcmInterface();

  /**
   * A callback used by DrakeLcmInterface::Subscribe(), with arguments:
   * - `message_buffer` A pointer to the byte vector that is the serial
   *   representation of the LCM message.
   * - `message_size` The size of `message_buffer`.
   *
   * A callback should never throw an exception, because it is indirectly
   * called from C functions.
   */
  using HandlerFunction = std::function<void(const void*, int)>;

  /**
   * Most users should use the drake::lcm::Publish() free function, instead of
   * this interface method.
   *
   * Publishes an LCM message on channel @p channel.
   *
   * @param channel The channel on which to publish the message.
   * Must not be the empty string.
   *
   * @param data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param data_size The length of @data in bytes.
   *
   * @param time_sec Time in seconds when the publish event occurred.
   * If unknown, use nullopt or a default-constructed optional.
   */
  virtual void Publish(const std::string& channel, const void* data,
                       int data_size, std::optional<double> time_sec) = 0;

  /**
   * Most users should use the drake::lcm::Subscribe() free function or the
   * drake::lcm::Subscriber wrapper class, instead of this interface method.
   *
   * Subscribes to an LCM channel without automatic message decoding. The
   * handler will be invoked when a message arrives on channel @p channel.
   *
   * The handler should never throw an exception, because it is indirectly
   * called from C functions.
   *
   * NOTE: Unlike upstream LCM, DrakeLcm does not support regexes for the
   * `channel` argument.
   *
   * @param channel The channel to subscribe to.
   * Must not be the empty string.
   *
   * @return the object used to manage the subscription if that is supported,
   * or else nullptr if not supported.  The unsubscribe-on-delete default is
   * `false`.  Refer to the DrakeSubscriptionInterface class overview for
   * details.
   */
  virtual std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string& channel, HandlerFunction) = 0;

  /**
   * Invokes the HandlerFunction callbacks for all subscriptions' pending
   * messages.  If @p timeout_millis is >0, blocks for up to that long until at
   * least one message is handled.
   * @return the number of messages handled, or 0 on timeout.
   * @throw std::exception when a subscribed handler throws.
   */
  virtual int HandleSubscriptions(int timeout_millis) = 0;

 protected:
  DrakeLcmInterface();

 private:
  // Allow our internal function to call the virtual function.
  friend void internal::OnHandleSubscriptionsError(
      DrakeLcmInterface* /* lcm */, const std::string& /* error_message */);
  // A virtual function to be called during HandleSubscriptions processing.
  virtual void OnHandleSubscriptionsError(const std::string& error_message) = 0;
};

/**
 * A helper class returned by DrakeLcmInterface::Subscribe() that allows for
 * (possibly automatic) unsubscription and/or queue capacity control.  Refer to
 * that method for additional details.
 *
 * Instance of this object are always stored in `std::shared_ptr` to manage
 * them as resources.  When a particular DrakeLcmInterface implementation does
 * not support subscription controls, the managed pointer will be `nullptr`
 * instead of an instance of this object.
 *
 * To unsubscribe, induce a call to the %DrakeSubscriptionInterface destructor
 * by bringing the `std::shared_ptr` use count to zero.  That usually means
 * either a call to `subscription.reset()` or by allowing it to go out of
 * scope.
 *
 * To *disable* unsubscription so that the pointer loss *never* causes
 * unsubscription, call `subscription->set_unsubscribe_on_delete(false)`.
 * To *enable* unsubscription, set it to `true`.  Which choice is active by
 * default is specified by whatever method returns this object.
 */
class DrakeSubscriptionInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeSubscriptionInterface)
  virtual ~DrakeSubscriptionInterface();

  /**
   * Sets whether or not the subscription on DrakeLcmInterface will be
   * terminated when this object is deleted.  It is permitted to call this
   * method many times, with a new `enabled` value each time.
   */
  virtual void set_unsubscribe_on_delete(bool enabled) = 0;

  /**
   * Sets this subscription's queue depth to store messages inbetween calls to
   * DrakeLcmInterface::HandleSubscriptions.  When the queue becomes full, new
   * received messages will be discarded.  The default depth is 1.
   *
   * @warning The memq:// LCM URL does not support per-channel queues, so this
   * method has no effect when memq is being used, e.g., in Drake unit tests.
   */
  virtual void set_queue_capacity(int capacity) = 0;

 protected:
  DrakeSubscriptionInterface();
};

/**
 * Publishes an LCM message on channel @p channel.
 *
 * @param lcm The LCM service on which to publish the message.
 * Must not be null.
 *
 * @param channel The channel on which to publish the message.
 * Must not be the empty string.
 *
 * @param message The message to publish.
 *
 * @param time_sec Time in seconds when the publish event occurred.
 * If unknown, use the default value of nullopt.
 */
template <typename Message>
void Publish(DrakeLcmInterface* lcm, const std::string& channel,
             const Message& message, std::optional<double> time_sec = {}) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
  const int num_bytes = message.getEncodedSize();
  DRAKE_THROW_UNLESS(num_bytes >= 0);
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  message.encode(bytes.data(), 0, num_bytes);
  lcm->Publish(channel, bytes.data(), num_bytes, time_sec);
}

/**
 * Subscribes to an LCM channel named @p channel and decodes messages of type
 * @p Message.  See also drake::lcm::Subscriber for a simple way to passively
 * observe received messages, without the need to write a handler function.
 *
 * @param lcm The LCM service on which to subscribe.
 * Must not be null.
 *
 * @param channel The channel on which to subscribe.
 * Must not be the empty string.
 *
 * @param handler The callback when a message is received and decoded without
 * error.
 *
 * @param on_error The callback when a message is received and cannot be
 * decoded; if no error handler is given, an exception is thrown instead.
 *
 * @return the object used to unsubscribe if that is supported, or else nullptr
 * if unsubscribe is not supported.  The unsubscribe-on-delete default is
 * `false`, so that ignoring this result leaves the subscription intact.  Refer
 * to the DrakeSubscriptionInterface class overview for details.
 *
 * @note Depending on the specific DrakeLcmInterface implementation, the
 * handler might be invoked on a different thread than this function.
 */
template <typename Message>
std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
    DrakeLcmInterface* lcm, const std::string& channel,
    std::function<void(const Message&)> handler,
    std::function<void()> on_error = {}) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
  auto result = lcm->Subscribe(channel, [=](const void* bytes, int size) {
    Message received{};
    const int size_decoded = received.decode(bytes, 0, size);
    if (size_decoded == size) {
      handler(received);
    } else if (on_error) {
      on_error();
    } else {
      // Register the error on the DrakeLcmInterface that owns us.  It will
      // throw once it's safe to do so (once C code is no longer on the stack).
      internal::OnHandleSubscriptionsError(
          lcm, "Error decoding message on " + channel);
    }
  });
  return result;
}

/**
 * Subscribes to and stores a copy of the most recent message on a given
 * channel, for some @p Message type.  All copies of a given Subscriber share
 * the same underlying data.  This class does NOT provide any mutex behavior
 * for multi-threaded locking; it should only be used in cases where the
 * governing DrakeLcmInterface::HandleSubscriptions is called from the same
 * thread that owns all copies of this object.
 */
template <typename Message>
class Subscriber final {
 public:
  // Intentionally copyable so that it can be returned and stored by-value.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Subscriber)

  /**
   * Subscribes to the (non-empty) @p channel on the given (non-null)
   * @p lcm instance.  The `lcm` pointer is only used during construction; it
   * is not retained by this object.  When a undecodable message is received,
   * @p on_error handler is invoked; when `on_error` is not provided, an
   * exception will be thrown instead.
   */
  Subscriber(DrakeLcmInterface* lcm, const std::string& channel,
             std::function<void()> on_error = {}) {
    subscription_ = drake::lcm::Subscribe<Message>(
        lcm, channel, [data = data_](const Message& message) {
          data->message = message;
          data->count++;
        }, std::move(on_error));
    if (subscription_) {
      subscription_->set_unsubscribe_on_delete(true);
    }
  }

  /**
   * Returns the most recently received message, or a value-initialized (zeros)
   * message otherwise.
   */
  const Message& message() const { return data_->message; }
  Message& message() { return data_->message; }

  /** Returns the total number of received messages. */
  int64_t count() const { return data_->count; }
  int64_t& count() { return data_->count; }

  /** Clears all data (sets the message and count to all zeros). */
  void clear() {
    data_->message = {};
    data_->count = 0;
  }

 private:
  struct Data {
    Message message{};
    int64_t count{0};
  };
  // Share a single copy of our (mutable) message storage, for all Subscribers
  // to view or modify *and* for our subscription closure to write into.  This
  // will not be destroyed until all Subscribers are gone AND the subscription
  // closure has been destroyed.
  std::shared_ptr<Data> data_{std::make_shared<Data>()};
  // Keep our subscription active as long as a copy of this Subscriber remains.
  std::shared_ptr<DrakeSubscriptionInterface> subscription_;
};

/// Convenience function that repeatedly calls `lcm->HandleSubscriptions()`
/// with a timeout value of `timeout_millis`, until `finished()` returns true.
/// Returns the total number of messages handled.
int LcmHandleSubscriptionsUntil(
    DrakeLcmInterface* lcm,
    const std::function<bool(void)>& finished,
    int timeout_millis = 100);

}  // namespace lcm
}  // namespace drake

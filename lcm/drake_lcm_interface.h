#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_nodiscard.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace lcm {

/**
 * For subscription management.
 * The destructor may be called from any thread.
 * The effect may not happen immediately.
 */
class DrakeSubscription {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeSubscription)
  virtual ~DrakeSubscription();

  /**
   * Sets whether or not the subscription on DrakeLcmInterface will be
   * terminated when this object is deleted.
   */
  virtual void set_unsubscribe_on_delete(bool enabled) = 0;

 protected:
  DrakeSubscription();
};

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
  virtual ~DrakeLcmInterface() = default;

  /**
   * A callback used by DrakeLcmInterface::Subscribe(), with arguments:
   * - `message_buffer` A pointer to the byte vector that is the serial
   *   representation of the LCM message.
   * - `message_size` The size of `message_buffer`.
   */
  using HandlerFunction = std::function<void(const void*, int)>;

  /**
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
   * If unknown, use drake::nullopt or a default-constructed optional.
   */
  virtual void Publish(const std::string& channel, const void* data,
                       int data_size, optional<double> time_sec) = 0;

  /**
   * Subscribes to an LCM channel without automatic message decoding. The
   * handler will be invoked when a message arrives on channel @p channel.
   *
   * @param channel The channel to subscribe to.
   * Must not be the empty string.
   *
   * @param queue_capacity The queue depth to store messages inbetween calls to
   * HandleSubscriptions.  When full, new received messages are immediately
   * discarded.
   */
  DRAKE_NODISCARD virtual std::unique_ptr<DrakeSubscription> Subscribe(
      const std::string& channel, int queue_capacity, HandlerFunction) = 0;

  /**
   * Invokes the HandlerFunction callbacks for all subscriptions' pending
   * messages.  If @p timeout_millis is >0, blocks for up to that long until at
   * least one message is handled.
   * @return the number of messages handled, or 0 on timeout.
   * @throw std::exception when a subscribed handler throws.
   */
  virtual int HandleSubscriptions(int timeout_millis) = 0;

 protected:
  DrakeLcmInterface() = default;
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
 * If unknown, use the default value of drake::nullopt.
 */
template <typename Message>
void Publish(DrakeLcmInterface* lcm, const std::string& channel,
             const Message& message, optional<double> time_sec = {}) {
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
 * observe received messages.
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
 * if unsubscribe is not supported.  Users may discard this value if the
 * subscription should be active as long as @p lcm is active.  To unsubscribe,
 * call result->set_unsubscribe_on_delete(true) and then manage the result as
 * resource.
 *
 * @note Depending on the specific DrakeLcmInterface implementation, the
 * handler might be invoked on a different thread than this function.
 */
template <typename Message>
std::unique_ptr<DrakeSubscription> Subscribe(
    DrakeLcmInterface* lcm, const std::string& channel,
    std::function<void(const Message&)> handler,
    std::function<void()> on_error = {}) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
  auto result = lcm->Subscribe(channel, 1, [=](const void* bytes, int size) {
    Message received{};
    const int size_decoded = received.decode(bytes, 0, size);
    if (size_decoded == size) {
      handler(received);
    } else if (on_error) {
      on_error();
    } else {
      throw std::runtime_error("Error decoding message on " + channel);
    }
  });
  if (result) {
    result->set_unsubscribe_on_delete(false);
  }
  return result;
}

/**
 * Subscribes to and stores a copy of the most recent message on a given
 * channel, for some @p Message type.  This class does NOT provide any mutex
 * behavior for multi-threaded locking; it should only be used in cases where
 * handlers are called from a thread other than the thread that owns this
 * object.  All copies of a given Subscriber share the same underlying data.
 */
template <typename Message>
class Subscriber final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Subscriber)

  /**
   * Subscribes to @p channel_name on the given @p lcm instance.  The `lcm`
   * pointer is aliased and must remain valid for the lifetime of this object.
   */
  Subscriber(DrakeLcmInterface* lcm, const std::string& channel_name,
             std::function<void()> on_error = {}) {
    subscription_ = drake::lcm::Subscribe<Message>(
        lcm, channel_name, [data = data_](const Message& message) {
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
  void reset() {
    data_->message_ = {};
    data_->count_ = 0;
  }

 private:
  struct Data {
    Message message{};
    int64_t count{0};
  };
  // Share a single copy of our (mutable) message storage, for all Subscribers
  // to view or modify *and* for our subscription closure to write into.  This
  // will not be destroyed until all Subscribers are gone AND the subscription
  // is Unsubscribed.
  std::shared_ptr<Data> data_{std::make_shared<Data>()};
  // Keep our subscription active as long as a copy of this Subscriber remains.
  std::shared_ptr<DrakeSubscription> subscription_;
};

}  // namespace lcm
}  // namespace drake

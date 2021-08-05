#include "drake/lcm/drake_lcm.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <utility>
#include <vector>

#include <glib.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/scope_exit.h"

namespace drake {
namespace lcm {
namespace {

// Keep this in sync with external/lcm/lcm.c.  Unfortunately, LCM does not
// appear to provide *any* API to determine this.
constexpr const char* const kLcmDefaultUrl = "udpm://239.255.76.67:7667?ttl=0";

// Defined below.
class DrakeSubscription;

}  // namespace

class DrakeLcm::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  explicit Impl(std::string lcm_url, bool defer_initialization)
      : requested_lcm_url_(std::move(lcm_url)),
        lcm_url_(requested_lcm_url_),
        deferred_initialization_(defer_initialization),
        lcm_(requested_lcm_url_) {
    // This duplicates logic from external/lcm/lcm.c, but until LCM offers an
    // API for this it's the best we can do.
    if (lcm_url_.empty()) {
      char* env_url = ::getenv("LCM_DEFAULT_URL");
      if (env_url) {
        lcm_url_ = env_url;
      }
      if (lcm_url_.empty()) {
        lcm_url_ = kLcmDefaultUrl;
      }
    }
  }

  // Housekeeping: scrub any deallocated subscriptions.
  void CleanUpOldSubscriptions() {
    subscriptions_.erase(std::remove_if(
        subscriptions_.begin(), subscriptions_.end(),
        [](const auto& weak_subscription) {
          return weak_subscription.expired();
        }), subscriptions_.end());
  }

  std::string requested_lcm_url_;
  std::string lcm_url_;
  bool deferred_initialization_{};
  ::lcm::LCM lcm_;
  std::vector<std::weak_ptr<DrakeSubscription>> subscriptions_;
  std::string handle_subscriptions_error_message_;
};

DrakeLcm::DrakeLcm() : DrakeLcm(std::string{}) {}

DrakeLcm::DrakeLcm(std::string lcm_url) : DrakeLcm(std::move(lcm_url), false) {}

DrakeLcm::DrakeLcm(std::string lcm_url, bool defer_initialization)
    : impl_(std::make_unique<Impl>(std::move(lcm_url), defer_initialization)) {
  if (!defer_initialization) {
    // Ensure that LCM's self-test happens deterministically (here in our
    // ctor) and NOT in the first HandleSubscriptions call.  Without this,
    // ThreadSanitizer builds may report false positives related to the
    // self-test happening concurrently with LCM publishing.
    impl_->lcm_.getFileno();
  }
}

std::string DrakeLcm::get_lcm_url() const {
  return impl_->lcm_url_;
}

::lcm::LCM* DrakeLcm::get_lcm_instance() {
  return &impl_->lcm_;
}

void DrakeLcm::Publish(const std::string& channel, const void* data,
                       int data_size, std::optional<double>) {
  DRAKE_THROW_UNLESS(!channel.empty());
  impl_->lcm_.publish(channel, data, data_size);
}

namespace {

// The concrete implementation of DrakeSubscriptionInterface used by DrakeLcm.
class DrakeSubscription final : public DrakeSubscriptionInterface {
 public:
  // We must disable copy/move/assign because we need our memory address to
  // remain stable; the native LCM stack keeps a pointer to this object.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeSubscription)

  using HandlerFunction = DrakeLcmInterface::HandlerFunction;
  using MultichannelHandlerFunction =
      DrakeLcmInterface::MultichannelHandlerFunction;

  static std::shared_ptr<DrakeSubscription> CreateSingleChannel(
      ::lcm::LCM* native_instance, const std::string& channel,
      HandlerFunction single_channel_handler) {
    // The argument to subscribeFunction is regex (not a string literal), so
    // we'll need to escape the channel name before calling subscribeFunction.
    char* const channel_regex = g_regex_escape_string(channel.c_str(), -1);
    ScopeExit guard([channel_regex](){ g_free(channel_regex); });

    return Create(native_instance, channel_regex,
                  [handler = std::move(single_channel_handler)](
                      std::string_view, const void* data, int size) {
                    handler(data, size);
                  });
  }

  static std::shared_ptr<DrakeSubscription> CreateMultichannel(
      ::lcm::LCM* native_instance,
      MultichannelHandlerFunction multichannel_handler) {
    return Create(native_instance, ".*", std::move(multichannel_handler));
  }

  static std::shared_ptr<DrakeSubscription> Create(
      ::lcm::LCM* native_instance, std::string_view channel_regex,
      MultichannelHandlerFunction handler) {
    DRAKE_DEMAND(native_instance != nullptr);
    DRAKE_DEMAND(handler != nullptr);

    // Create the result.
    auto result = std::make_shared<DrakeSubscription>();
    result->channel_regex_ = channel_regex;
    result->native_instance_ = native_instance;
    result->user_callback_ = std::move(handler);
    result->weak_self_reference_ = result;
    result->strong_self_reference_ = result;

    // Sanity checks.  (The use_count will be 2 because both 'result' and
    // 'strong_self_reference' keep the subscription alive.)
    DRAKE_DEMAND(result->user_callback_ != nullptr);
    DRAKE_DEMAND(result->weak_self_reference_.use_count() == 2);
    DRAKE_DEMAND(result->strong_self_reference_.use_count() == 2);
    DRAKE_DEMAND(result->strong_self_reference_ != nullptr);

    return result;
  }

  ~DrakeSubscription() {
    DRAKE_DEMAND(strong_self_reference_ == nullptr);
    if (native_subscription_) {
      DRAKE_DEMAND(native_instance_ != nullptr);
      native_instance_->unsubscribe(native_subscription_);
    }
  }

  void set_unsubscribe_on_delete(bool enabled) final {
    DRAKE_DEMAND(!weak_self_reference_.expired());
    if (enabled) {
      // The caller needs to keep this Subscription active.
      strong_self_reference_.reset();
    } else {
      // This DrakeSubscription will keep itself active.
      strong_self_reference_ = weak_self_reference_.lock();
    }
  }

  void set_queue_capacity(int capacity) final {
    DRAKE_DEMAND(!weak_self_reference_.expired());
    queue_capacity_ = capacity;
    if (native_subscription_) {
      DRAKE_DEMAND(native_instance_ != nullptr);
      native_subscription_->setQueueCapacity(capacity);
    }
  }

  void AttachIfNeeded() {
    if (native_subscription_ != nullptr) {
      return;
    }
    native_subscription_ = native_instance_->subscribeFunction(
        channel_regex_, &DrakeSubscription::NativeCallback, this);
    native_subscription_->setQueueCapacity(queue_capacity_);
  }

  // This is ONLY called from the DrakeLcm dtor.  Thus, a HandleSubscriptions
  // is never in flight, so we can freely change any/all of our member fields.
  void Detach() {
    DRAKE_DEMAND(!weak_self_reference_.expired());
    if (native_subscription_) {
      DRAKE_DEMAND(native_instance_ != nullptr);
      native_instance_->unsubscribe(native_subscription_);
    }
    native_instance_ = {};
    native_subscription_ = {};
    user_callback_ = {};
    weak_self_reference_ = {};
    strong_self_reference_ = {};
  }

  // The native LCM stack calls into here.
  static void NativeCallback(
      const ::lcm::ReceiveBuffer* buffer,
      const std::string& channel,
      DrakeSubscription* self) {
    DRAKE_DEMAND(buffer != nullptr);
    DRAKE_DEMAND(self != nullptr);
    self->InstanceCallback(channel, buffer);
  }

 private:
  struct AsIfPrivateConstructor {};

 public:
  // Only use Create; don't call this directly.  (We use a private struct to
  // prevent anyone but this class from calling the ctor.)
  explicit DrakeSubscription(AsIfPrivateConstructor = {}) {}

 private:
  void InstanceCallback(const std::string& channel,
                        const ::lcm::ReceiveBuffer* buffer) {
    DRAKE_DEMAND(!weak_self_reference_.expired());
    if (user_callback_ != nullptr) {
      user_callback_(channel, buffer->data, buffer->data_size);
    }
  }

  std::string channel_regex_;

  // The native handle we can use to unsubscribe.
  ::lcm::LCM* native_instance_{};
  ::lcm::Subscription* native_subscription_{};
  int queue_capacity_{1};

  DrakeLcmInterface::MultichannelHandlerFunction user_callback_;

  // We can use "strong" to pretend a subscriber is still active.
  std::weak_ptr<DrakeSubscriptionInterface> weak_self_reference_;
  std::shared_ptr<DrakeSubscriptionInterface> strong_self_reference_;
};

}  // namespace

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  DRAKE_THROW_UNLESS(handler != nullptr);
  impl_->CleanUpOldSubscriptions();

  // Add the new subscriber.
  auto result = DrakeSubscription::CreateSingleChannel(
      &(impl_->lcm_), channel, std::move(handler));
  if (!impl_->deferred_initialization_) {
    result->AttachIfNeeded();
  }
  impl_->subscriptions_.push_back(result);
  DRAKE_DEMAND(!impl_->subscriptions_.back().expired());
  return result;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::SubscribeAllChannels(
    MultichannelHandlerFunction handler) {
  DRAKE_THROW_UNLESS(handler != nullptr);
  impl_->CleanUpOldSubscriptions();

  // Add the new subscriber.
  auto result = DrakeSubscription::CreateMultichannel(
      &(impl_->lcm_), std::move(handler));
  if (!impl_->deferred_initialization_) {
    result->AttachIfNeeded();
  }
  impl_->subscriptions_.push_back(result);
  DRAKE_DEMAND(!impl_->subscriptions_.back().expired());
  return result;
}

int DrakeLcm::HandleSubscriptions(int timeout_millis) {
  if (impl_->deferred_initialization_) {
    for (auto& sub : impl_->subscriptions_) {
      sub.lock()->AttachIfNeeded();
    }
    impl_->deferred_initialization_ = false;
  }
  // Keep pumping handleTimeout until it's empty, but only pause for the
  // timeout on the first attempt.
  int total_messages = 0;
  int zero_or_one = impl_->lcm_.handleTimeout(timeout_millis);
  for (; zero_or_one > 0; zero_or_one = impl_->lcm_.handleTimeout(0)) {
    DRAKE_DEMAND(zero_or_one == 1);
    ++total_messages;
  }
  // If a handler posted an error, raise it now that we're done with LCM C code.
  if (!impl_->handle_subscriptions_error_message_.empty()) {
    std::string message = std::move(impl_->handle_subscriptions_error_message_);
    impl_->handle_subscriptions_error_message_ = {};
    throw std::runtime_error(std::move(message));
  }
  return total_messages;
}

void DrakeLcm::OnHandleSubscriptionsError(const std::string& error_message) {
  DRAKE_DEMAND(!error_message.empty());
  // Stash the exception message for later.  This is "last one wins" if there
  // are multiple errors.  We can only throw one anyway, and doesn't matter
  // which one we throw.
  impl_->handle_subscriptions_error_message_ = error_message;
}

DrakeLcm::~DrakeLcm() {
  // Invalidate our DrakeSubscription objects.
  for (const auto& weak_subscription : impl_->subscriptions_) {
    auto subscription = weak_subscription.lock();
    if (subscription) {
      subscription->Detach();
    }
  }
}

}  // namespace lcm
}  // namespace drake

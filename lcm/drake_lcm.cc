#include "drake/lcm/drake_lcm.h"

#include <algorithm>
#include <cstdlib>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

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

  explicit Impl(std::string lcm_url)
      : requested_lcm_url_(std::move(lcm_url)),
        lcm_url_(requested_lcm_url_),
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

  std::string requested_lcm_url_;
  std::string lcm_url_;
  ::lcm::LCM lcm_;
  std::vector<std::weak_ptr<DrakeSubscription>> subscriptions_;
};

DrakeLcm::DrakeLcm() : DrakeLcm(std::string{}) {}

DrakeLcm::DrakeLcm(std::string lcm_url)
    : impl_(std::make_unique<Impl>(std::move(lcm_url))) {
  // Ensure that LCM's self-test happens deterministically (here in our ctor),
  // and NOT in the receive thread or first HandleSubscriptions call.  Without
  // this, ThreadSanitizer builds may report false positives related to the
  // self-test happening concurrently with the LCM publishing.
  impl_->lcm_.getFileno();
}

std::string DrakeLcm::get_lcm_url() const {
  return impl_->lcm_url_;
}

::lcm::LCM* DrakeLcm::get_lcm_instance() {
  return &impl_->lcm_;
}

void DrakeLcm::Publish(const std::string& channel, const void* data,
                       int data_size, optional<double>) {
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

  static std::shared_ptr<DrakeSubscription> Create(
      ::lcm::LCM* native_instance, const std::string& channel,
      HandlerFunction handler) {
    DRAKE_DEMAND(native_instance != nullptr);

    // Create the result.
    auto result = std::make_shared<DrakeSubscription>();
    result->native_instance_ = native_instance;
    result->user_callback_ = std::move(handler);
    result->weak_self_reference_ = result;
    result->strong_self_reference_ = result;
    result->native_subscription_ = native_instance->subscribeFunction(
      channel, &DrakeSubscription::NativeCallback, result.get());
    result->native_subscription_->setQueueCapacity(1);

    // Sanity checks.  (The use_count will be 2 because both 'result' and
    // 'strong_self_reference' keep the subscription alive.)
    DRAKE_DEMAND(result->native_instance_ != nullptr);
    DRAKE_DEMAND(result->native_subscription_ != nullptr);
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
    if (native_subscription_) {
      DRAKE_DEMAND(native_instance_ != nullptr);
      native_subscription_->setQueueCapacity(capacity);
    }
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
      const std::string& /* channel */ ,
      DrakeSubscription* self) {
    DRAKE_DEMAND(buffer != nullptr);
    DRAKE_DEMAND(self != nullptr);
    self->InstanceCallback(buffer);
  }

 private:
  struct AsIfPrivateConstructor {};

 public:
  // Only use Create; don't call this directly.  (We use a private struct to
  // prevent anyone but this class from calling the ctor.)
  explicit DrakeSubscription(AsIfPrivateConstructor = {}) {}

 private:
  void InstanceCallback(const ::lcm::ReceiveBuffer* buffer) {
    DRAKE_DEMAND(!weak_self_reference_.expired());
    if (user_callback_ != nullptr) {
      user_callback_(buffer->data, buffer->data_size);
    }
  }

  // The native handle we can use to unsubscribe.
  ::lcm::LCM* native_instance_{};
  ::lcm::Subscription* native_subscription_{};

  // The user's function that handles raw message data.
  DrakeLcmInterface::HandlerFunction user_callback_;

  // We can use "strong" to pretend a subscriber is still active.
  std::weak_ptr<DrakeSubscriptionInterface> weak_self_reference_;
  std::shared_ptr<DrakeSubscriptionInterface> strong_self_reference_;
};

}  // namespace

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  DRAKE_THROW_UNLESS(handler != nullptr);

  // Some housekeeping: scrub any deallocated subscribers.
  auto& subs = impl_->subscriptions_;
  subs.erase(std::remove_if(
      subs.begin(), subs.end(),
      [](const auto& weak_subscription) {
        return weak_subscription.expired();
      }), subs.end());

  // Add the new subscriber.
  auto result = DrakeSubscription::Create(
      &(impl_->lcm_), channel, std::move(handler));
  subs.push_back(result);
  DRAKE_DEMAND(!subs.back().expired());
  return result;
}

int DrakeLcm::HandleSubscriptions(int timeout_millis) {
  // Keep pumping handleTimeout until it's empty, but only pause for the
  // timeout on the first attempt.
  int total_messages = 0;
  int zero_or_one = impl_->lcm_.handleTimeout(timeout_millis);
  for (; zero_or_one > 0; zero_or_one = impl_->lcm_.handleTimeout(0)) {
    DRAKE_DEMAND(zero_or_one == 1);
    ++total_messages;
  }
  return total_messages;
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

#include "drake/lcm/drake_lcm.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/lcm/lcm_receive_thread.h"

namespace drake {
namespace lcm {
namespace {

// Keep this in sync with lcm_receive_thread.cc.
const int kMagicTimeoutMillis = 247;

// Keep this in sync with external/lcm/lcm.c.
constexpr const char* const kLcmDefaultUrl = "udpm://239.255.76.67:7667?ttl=0";

// Dummy type to know if the subscription is still active.
struct SubscriberTag {};

struct SubscriptionData {
  // We need our memory address to remain stable; the native LCM stack keeps a
  // pointer to this object.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SubscriptionData)
  SubscriptionData() = default;

  // The native handle we can use to unsubscribe.
  ::lcm::Subscription* native_subscription{};

  // The user's function that handles raw message data.
  DrakeLcmInterface::HandlerFunction user_callback;

  // If this is expired, then no subscriber is still active.
  std::weak_ptr<SubscriberTag> has_subscriber;

  // We can set this to pretend a subscribe is still active.
  std::shared_ptr<SubscriberTag> forced_has_subscriber;
};

void Callback(const ::lcm::ReceiveBuffer* buffer,
              const std::string& /* channel */ ,
              SubscriptionData* context) {
  DRAKE_DEMAND(buffer != nullptr);
  DRAKE_DEMAND(context != nullptr);
  if (!context->has_subscriber.expired()) {
    (context->user_callback)(buffer->data, buffer->data_size);
  }
}

}  // namespace

class DrakeLcm::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  explicit Impl(std::string lcm_url)
      : requested_lcm_url_(std::move(lcm_url)),
        lcm_url_(requested_lcm_url_),
        lcm_(requested_lcm_url_) {
    if (lcm_url_.empty()) {
      lcm_url_ = kLcmDefaultUrl;
      // XXX Need to check the envvar.
    }
  }

  std::string requested_lcm_url_;
  std::string lcm_url_;
  ::lcm::LCM lcm_;
  std::vector<std::shared_ptr<SubscriptionData>> subscriptions_;

  // Deprecated.
  std::unique_ptr<LcmReceiveThread> receive_thread_;
};

DrakeLcm::DrakeLcm() : DrakeLcm(std::string{}) {}

DrakeLcm::DrakeLcm(std::string lcm_url)
    : impl_(std::make_unique<Impl>(std::move(lcm_url))) {
  // Ensure that LCM's self-test happens deterministically (here, in our ctor),
  // and NOT in the receive thread or first HandleSubscriptions call.  Without
  // this, ThreadSanitizer builds may report false positives related to the
  // self-test happening concurrently with the LCM publishing.
  impl_->lcm_.getFileno();
}

DrakeLcm::~DrakeLcm() {
  // Stop invoking the subscriptions, prior to destroying them.
  impl_->receive_thread_.reset();
}

// Deprecated.
void DrakeLcm::StartReceiveThread() {
  DRAKE_DEMAND(impl_->receive_thread_ == nullptr);
  impl_->receive_thread_ = std::make_unique<LcmReceiveThread>(this);
}

// Deprecated.
void DrakeLcm::StopReceiveThread() {
  if (impl_->receive_thread_ != nullptr) {
    impl_->receive_thread_->Stop();
    impl_->receive_thread_.reset();
  }
}

// Deprecated.
bool DrakeLcm::IsReceiveThreadRunning() const {
  return impl_->receive_thread_ != nullptr;
}

// Deprecated.
std::string DrakeLcm::get_requested_lcm_url() const {
  return impl_->requested_lcm_url_;
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
class ConcreteSubscription final : public DrakeSubscription {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConcreteSubscription)

  explicit ConcreteSubscription(SubscriptionData* data)
      : tag_(std::make_shared<SubscriberTag>()),
        data_(data) {
    DRAKE_DEMAND(data != nullptr);
    data_->has_subscriber = tag_;
  }

  void set_unsubscribe_on_delete(bool enabled) final {
    if (enabled) {
      // Only our tag_ will keep the SubscriberData active.
      data_->forced_has_subscriber.reset();
    } else {
      // The SubscriberData will keep itself active.
      data_->forced_has_subscriber = tag_;
    }
  }

 private:
  const std::shared_ptr<SubscriberTag> tag_;
  SubscriptionData* const data_;
};
}  // namespace

std::unique_ptr<DrakeSubscription> DrakeLcm::Subscribe(
    const std::string& channel, int queue_capacity, HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  DRAKE_THROW_UNLESS(queue_capacity >= 1);

  auto data = std::make_shared<SubscriptionData>();
  auto result = std::make_unique<ConcreteSubscription>(data.get());
  impl_->subscriptions_.push_back(data);

  data->user_callback = std::move(handler);
  data->native_subscription = impl_->lcm_.subscribeFunction(
      channel, &Callback, data.get());
  data->native_subscription->setQueueCapacity(queue_capacity);

  return result;
}

int DrakeLcm::HandleSubscriptions(int timeout_millis) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (IsReceiveThreadRunning()) {
    // Only *our* receive thread should be dispatching subscriptions.
    DRAKE_THROW_UNLESS(timeout_millis == kMagicTimeoutMillis);
  }
#pragma GCC diagnostic pop

  // Keep pumping handleTimeout until its empty, but only pause for the timeout
  // on the first attempt.
  int total_messages = 0;
  int single_handle = impl_->lcm_.handleTimeout(timeout_millis);
  while (single_handle > 0) {
    DRAKE_DEMAND(single_handle == 1);
    ++total_messages;
    single_handle = impl_->lcm_.handleTimeout(0);
  }

  // Remove any unwanted subscribers.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (!IsReceiveThreadRunning()) {
#pragma GCC diagnostic pop
    auto& subs = impl_->subscriptions_;
    subs.erase(std::remove_if(subs.begin(), subs.end(), [](const auto& data) {
      return data->has_subscriber.expired();
    }), subs.end());
  }

  return total_messages;
}

}  // namespace lcm
}  // namespace drake

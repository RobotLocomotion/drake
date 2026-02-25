#include "drake/lcm/drake_lcm.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <utility>
#include <vector>

#include <glib.h>
#include <lcm/lcm.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/network_policy.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace lcm {
namespace {

// Keep this in sync with external/lcm/lcm.c.  Unfortunately, LCM does not
// appear to provide *any* API to determine this.
constexpr const char* const kLcmDefaultUrl = "udpm://239.255.76.67:7667?ttl=0";

// Defined below.
class DrakeSubscription;

void ThrowIfLcmError(bool is_lcm_error) {
  if (is_lcm_error) {
    throw std::logic_error(
        "LCM has encountered an error. The most likely cause is network "
        "misconfiguration. See "
        "https://drake.mit.edu/troubleshooting.html#lcm-macos and "
        "https://lcm-proj.github.io/lcm/content/multicast-setup.html for "
        "more information.");
  }
}

}  // namespace

class DrakeLcm::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);

  explicit Impl(const DrakeLcmParams& params)
      : requested_lcm_url_(params.lcm_url),
        deferred_initialization_(params.defer_initialization),
        channel_suffix_(params.channel_suffix) {
    // This duplicates logic from external/lcm/lcm.c, but until LCM offers an
    // API for this it's the best we can do.
    lcm_url_ = requested_lcm_url_;
    if (lcm_url_.empty()) {
      char* env_url = ::getenv("LCM_DEFAULT_URL");
      if (env_url) {
        lcm_url_ = env_url;
      }
      if (lcm_url_.empty()) {
        lcm_url_ = kLcmDefaultUrl;
      }
    }
    // Check DRAKE_ALLOW_NETWORK.
    if (lcm_url_.substr(0, 7) != "memq://") {
      if (!drake::internal::IsNetworkingAllowed("lcm")) {
        throw std::runtime_error(fmt::format(
            "LCM URL {} has been disabled via the DRAKE_ALLOW_NETWORK "
            "environment variable",
            lcm_url_));
      }
    }
    // Create the native instance only after all other checks are finished.
    lcm_ = ::lcm_create(lcm_url_.c_str());
    if (lcm_ == nullptr) {
      // The initialization failed and printed a console warning. Create
      // a dummy object instead (to at least have something non-null).
      lcm_ = ::lcm_create("memq://");
      DRAKE_THROW_UNLESS(lcm_ != nullptr);
    }
  }

  // Housekeeping: scrub any deallocated subscriptions.
  void CleanUpOldSubscriptions() {
    subscriptions_.erase(
        std::remove_if(subscriptions_.begin(), subscriptions_.end(),
                       [](const auto& weak_subscription) {
                         return weak_subscription.expired();
                       }),
        subscriptions_.end());
  }

  const std::string requested_lcm_url_;
  std::string lcm_url_;
  bool deferred_initialization_{};
  lcm_t* lcm_{};
  const std::string channel_suffix_;
  std::vector<std::weak_ptr<DrakeSubscription>> subscriptions_;
  std::string handle_subscriptions_error_message_;
};

DrakeLcm::DrakeLcm() : DrakeLcm(std::string{}) {}

DrakeLcm::DrakeLcm(std::string lcm_url)
    : DrakeLcm(DrakeLcmParams{.lcm_url = std::move(lcm_url)}) {}

DrakeLcm::DrakeLcm(const DrakeLcmParams& params)
    : impl_(std::make_unique<Impl>(params)) {
  if (!params.defer_initialization) {
    // Ensure that LCM's self-test happens deterministically (here in our
    // ctor) and NOT in the first HandleSubscriptions call.  Without this,
    // ThreadSanitizer builds may report false positives related to the
    // self-test happening concurrently with LCM publishing.
    ThrowIfLcmError(::lcm_get_fileno(impl_->lcm_) < 0);
  }
}

std::string DrakeLcm::get_lcm_url() const {
  return impl_->lcm_url_;
}

void* DrakeLcm::get_native_lcm_handle_for_unit_testing() {
  return impl_->lcm_;
}

void DrakeLcm::Publish(const std::string& channel, const void* data,
                       int data_size, std::optional<double>) {
  DRAKE_THROW_UNLESS(!channel.empty());
  if (impl_->channel_suffix_.empty()) {
    ::lcm_publish(impl_->lcm_, channel.c_str(), data, data_size);
  } else {
    const std::string actual_channel = channel + impl_->channel_suffix_;
    ::lcm_publish(impl_->lcm_, actual_channel.c_str(), data, data_size);
  }
}

namespace {

// Given a literal string, escape it to be safe to use in an LCM channel regex.
// For example ".foo" should be escaped to "\.foo" so that it only matches the
// exact literal string, not "xfoo".
std::string ConvertLiteralStringToLcmRegex(const std::string& literal) {
  char* const result_cstr = g_regex_escape_string(literal.c_str(), -1);
  const std::string result{result_cstr};
  g_free(result_cstr);
  return result;
}

// The concrete implementation of DrakeSubscriptionInterface used by DrakeLcm.
class DrakeSubscription final : public DrakeSubscriptionInterface {
 public:
  // We must disable copy/move/assign because we need our memory address to
  // remain stable; the native LCM stack keeps a pointer to this object.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeSubscription);

  using HandlerFunction = DrakeLcmInterface::HandlerFunction;
  using MultichannelHandlerFunction =
      DrakeLcmInterface::MultichannelHandlerFunction;

  static std::shared_ptr<DrakeSubscription> CreateSingleChannel(
      ::lcm_t* native_instance, const std::string& channel,
      HandlerFunction single_channel_handler) {
    // N.B. The argument to CreateMultichannel is regex, so we need to escape
    // the channel name as part delegating to it.
    return CreateMultichannel(
        native_instance, ConvertLiteralStringToLcmRegex(channel),
        [handler = std::move(single_channel_handler)](
            std::string_view, const void* data, int size) {
          handler(data, size);
        });
  }

  static std::shared_ptr<DrakeSubscription> CreateMultichannel(
      ::lcm_t* native_instance, std::string_view channel_regex,
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
      ::lcm_unsubscribe(native_instance_, native_subscription_);
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
      ::lcm_subscription_set_queue_capacity(native_subscription_,
                                            queue_capacity_);
    }
  }

  void AttachIfNeeded() {
    if (native_subscription_ != nullptr) {
      return;
    }
    native_subscription_ =
        ::lcm_subscribe(native_instance_, channel_regex_.c_str(),
                        &DrakeSubscription::NativeCallback, this);
    ThrowIfLcmError(native_subscription_ == nullptr);
    set_queue_capacity(queue_capacity_);
  }

  // This is ONLY called from the DrakeLcm dtor.  Thus, a HandleSubscriptions
  // is never in flight, so we can freely change any/all of our member fields.
  void Detach() {
    DRAKE_DEMAND(!weak_self_reference_.expired());
    if (native_subscription_) {
      DRAKE_DEMAND(native_instance_ != nullptr);
      ::lcm_unsubscribe(native_instance_, native_subscription_);
    }
    native_instance_ = {};
    native_subscription_ = {};
    user_callback_ = {};
    weak_self_reference_ = {};
    strong_self_reference_ = {};
  }

 private:
  struct AsIfPrivateConstructor {};

 public:
  // Only use Create; don't call this directly.  (We use a private struct to
  // prevent anyone but this class from calling the ctor.)
  explicit DrakeSubscription(AsIfPrivateConstructor = {}) {}

 private:
  // The native LCM stack calls into here.
  static void NativeCallback(const ::lcm_recv_buf_t* buffer,
                             const char* channel, void* user_data) {
    DRAKE_DEMAND(buffer != nullptr);
    DRAKE_DEMAND(channel != nullptr);
    DRAKE_DEMAND(user_data != nullptr);
    auto* self = static_cast<DrakeSubscription*>(user_data);
    DRAKE_DEMAND(!self->weak_self_reference_.expired());
    if (self->user_callback_ != nullptr) {
      self->user_callback_(channel, buffer->data, buffer->data_size);
    }
  }

  std::string channel_regex_;

  // The native handle we can use to unsubscribe.
  ::lcm_t* native_instance_{};
  ::lcm_subscription_t* native_subscription_{};
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
  const std::string actual_channel = channel + impl_->channel_suffix_;
  auto result = DrakeSubscription::CreateSingleChannel(
      impl_->lcm_, actual_channel, std::move(handler));
  if (!impl_->deferred_initialization_) {
    result->AttachIfNeeded();
  }
  impl_->subscriptions_.push_back(result);
  DRAKE_DEMAND(!impl_->subscriptions_.back().expired());
  return result;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::SubscribeMultichannel(
    std::string_view regex, MultichannelHandlerFunction handler) {
  DRAKE_THROW_UNLESS(!regex.empty());
  DRAKE_THROW_UNLESS(handler != nullptr);
  impl_->CleanUpOldSubscriptions();

  const std::string& suffix = impl_->channel_suffix_;
  if (!suffix.empty()) {
    handler = [&suffix, handler](std::string_view channel, const void* data,
                                 int length) {
      // TODO(ggould-tri) Use string_view::ends_with() once we have C++20.
      DRAKE_DEMAND(channel.length() >= suffix.length() &&
                   channel.substr(channel.length() - suffix.length()) ==
                       suffix);
      channel.remove_suffix(suffix.length());
      handler(channel, data, length);
    };
  }

  // Add the new subscriber.
  auto result = DrakeSubscription::CreateMultichannel(
      impl_->lcm_, std::string(regex) + ConvertLiteralStringToLcmRegex(suffix),
      std::move(handler));
  if (!impl_->deferred_initialization_) {
    result->AttachIfNeeded();
  }
  impl_->subscriptions_.push_back(result);
  DRAKE_DEMAND(!impl_->subscriptions_.back().expired());
  return result;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::SubscribeAllChannels(
    MultichannelHandlerFunction handler) {
  return SubscribeMultichannel(".*", std::move(handler));
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
  int zero_or_one = ::lcm_handle_timeout(impl_->lcm_, timeout_millis);
  for (; zero_or_one > 0; zero_or_one = ::lcm_handle_timeout(impl_->lcm_, 0)) {
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
  ::lcm_destroy(impl_->lcm_);
}

bool DrakeLcm::available() {
  return true;
}

}  // namespace lcm
}  // namespace drake

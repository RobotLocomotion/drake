#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <functional>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;
using std::make_unique;

namespace {
constexpr int kStateIndexMessage = 0;
constexpr int kStateIndexMessageCount = 1;
constexpr int kMagic = 6832;  // An arbitrary value.
}  // namespace

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    std::shared_ptr<const SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm,
    double wait_for_message_on_initialization_timeout)
    : channel_(channel),
      serializer_(std::move(serializer)),
      magic_number_{kMagic},
      // Only capture the lcm pointer if it is required.
      lcm_{wait_for_message_on_initialization_timeout > 0 ? lcm : nullptr},
      wait_for_message_on_initialization_timeout_{
          wait_for_message_on_initialization_timeout} {
  DRAKE_THROW_UNLESS(serializer_ != nullptr);
  DRAKE_THROW_UNLESS(wait_for_message_on_initialization_timeout_ <= 0 ||
                     lcm_ != nullptr);
  DRAKE_THROW_UNLESS(!std::isnan(wait_for_message_on_initialization_timeout));

  subscription_ = lcm->Subscribe(
      channel_, [this](const void* buffer, int size) {
        this->HandleMessage(buffer, size);
      });
  if (subscription_) {
    subscription_->set_unsubscribe_on_delete(true);
  }

  // Declare our two states (message_value, message_count).
  static_assert(kStateIndexMessage == 0, "");
  auto message_state_index =
      this->DeclareAbstractState(*serializer_->CreateDefaultValue());
  static_assert(kStateIndexMessageCount == 1, "");
  this->DeclareAbstractState(Value<int>(0));

  // Our sole output is the message state.
  this->DeclareStateOutputPort(kUseDefaultName, message_state_index);

  // Declare an unrestricted forced update handler that is invoked when a
  // "forced" trigger occurs. This gives the user flexibility to force update
  // the abstract states.
  // TODO(jwnimmer-tri) We provide forced unrestricted update handling for
  // backwards compatibility reasons.  We need to decide if forced triggers
  // actually make sense for this system, and if not then deprecate and
  // remove this feature.
  this->DeclareForcedUnrestrictedUpdateEvent(
      &LcmSubscriberSystem::ProcessMessageAndStoreToAbstractState);

  // On initialization, we process any existing received messages and maybe
  // wait for new messages.
  this->DeclareInitializationUnrestrictedUpdateEvent(
      &LcmSubscriberSystem::Initialize);

  set_name(make_name(channel_));
}

LcmSubscriberSystem::~LcmSubscriberSystem() {
  // Violate our class invariant, to help catch use-after-free.
  magic_number_ = 0;
}

// This function processes the internal received message and store the results
// to the abstract states, which include both the message and message counts.
EventStatus LcmSubscriberSystem::ProcessMessageAndStoreToAbstractState(
    const Context<double>& context, State<double>* state) const {
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  const int context_message_count = GetMessageCount(context);
  if (context_message_count == received_message_count_) {
    state->SetFrom(context.get_state());
    return EventStatus::DidNothing();
  }
  serializer_->Deserialize(
      received_message_.data(), received_message_.size(),
      &state->get_mutable_abstract_state().get_mutable_value(
          kStateIndexMessage));
  state->get_mutable_abstract_state<int>(kStateIndexMessageCount) =
      received_message_count_;
  return EventStatus::Succeeded();
}

int LcmSubscriberSystem::GetMessageCount(const Context<double>& context) const {
  return context.get_abstract_state<int>(kStateIndexMessageCount);
}

// Adds additional event scheduling to the default implementation:
// if a new message has been received, adds an event trigger scheduled
// for the current time so it will be handled immediately.
void LcmSubscriberSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  // Do nothing unless we have a new message.
  const int last_message_count = GetMessageCount(context);
  const int received_message_count = [this]() {
    std::unique_lock<std::mutex> lock(received_message_mutex_);
    return received_message_count_;
  }();
  if (last_message_count == received_message_count) {
    return;
  }

  // Create a unrestricted event and tie the handler to the corresponding
  // function.
  auto callback = [](const System<double>& system,
                     const Context<double>& callback_context,
                     const systems::UnrestrictedUpdateEvent<double>&,
                     State<double>* callback_state) {
    const auto& self = dynamic_cast<const LcmSubscriberSystem&>(system);
    return self.ProcessMessageAndStoreToAbstractState(callback_context,
                                                      callback_state);
  };

  // Schedule an update event at the current time.
  *time = context.get_time();
  EventCollection<UnrestrictedUpdateEvent<double>>& uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.AddEvent(
      systems::UnrestrictedUpdateEvent<double>(
          TriggerType::kTimed, callback));
}

std::string LcmSubscriberSystem::make_name(const std::string& channel) {
  return "LcmSubscriberSystem(" + channel + ")";
}

const std::string& LcmSubscriberSystem::get_channel_name() const {
  return channel_;
}

void LcmSubscriberSystem::HandleMessage(const void* buffer, int size) {
  DRAKE_LOGGER_TRACE("Receiving LCM {} message", channel_);
  DRAKE_DEMAND(magic_number_ == kMagic);

  const uint8_t* const rbuf_begin = static_cast<const uint8_t*>(buffer);
  const uint8_t* const rbuf_end = rbuf_begin + size;
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  received_message_.clear();
  received_message_.insert(received_message_.begin(), rbuf_begin, rbuf_end);
  received_message_count_++;
  received_message_condition_variable_.notify_all();
}

int LcmSubscriberSystem::WaitForMessage(
    int old_message_count, AbstractValue* message, double timeout) const {
// NOLINTNEXTLINE(build/namespaces): The chrono literals are just so convenient.
  using namespace std::chrono_literals;
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  // The message buffer and counter are updated in HandleMessage(), which is
  // a callback function invoked by a different thread owned by the
  // drake::lcm::DrakeLcmInterface instance passed to the constructor. Thus,
  // for thread safety, these need to be properly protected by a mutex.
  std::unique_lock<std::mutex> lock(received_message_mutex_);

  // Predicate to handle spurious wakeup -- in other words, we can stop if we
  // detect a message has *actually* been received.
  auto message_received = [&]() {
    return received_message_count_ > old_message_count;
  };
  if (timeout <= 0) {
    // No timeout.
    received_message_condition_variable_.wait(lock, message_received);
  } else {
    // With timeout.

    const Duration requested = std::chrono::duration_cast<Duration>(
        std::chrono::duration<double>(timeout));
    // Hour-encoding of ten years. Empirical evidence suggests that
    // condition_variable::wait_* doesn't work over the full domain of otherwise
    // valid Duration values (i.e., it can be "too large"). So, for
    // exceptionally "large" timeouts, we cap it, silently, at ten years; if
    // this proves too short for any given process, we can modify it later.
    const Duration kMaxDuration(87600h);
    const Duration duration =
        requested < kMaxDuration ? requested : kMaxDuration;
    DRAKE_ASSERT(TimePoint::max() - duration > Clock::now());
    if (!received_message_condition_variable_.wait_for(lock, duration,
                                                       message_received)) {
      return received_message_count_;
    }
  }

  if (message) {
      serializer_->Deserialize(
          received_message_.data(), received_message_.size(), message);
  }

  return received_message_count_;
}

int LcmSubscriberSystem::GetInternalMessageCount() const {
  std::unique_lock<std::mutex> lock(received_message_mutex_);
  return received_message_count_;
}

EventStatus LcmSubscriberSystem::Initialize(const Context<double>& context,
                                            State<double>* state) const {
  // In the default case when waiting is disabled, we'll opportunistically try
  // to update our state, but we might return EventStatus::DidNothing().
  if (wait_for_message_on_initialization_timeout_ <= 0.0) {
    return ProcessMessageAndStoreToAbstractState(context, state);
  }

  // The user has requested to pause initialization until context changes.
  // Start by peeking to see if there's already a message waiting.
  DRAKE_DEMAND(lcm_ != nullptr);
  lcm_->HandleSubscriptions(0 /* timeout_millis */);
  EventStatus result = ProcessMessageAndStoreToAbstractState(context, state);
  if (result.severity() != EventStatus::kDidNothing) {
    return result;
  }

  // No message was pending. We'll spin until we get one (or run out of time).
  log()->info("Waiting for messages on {}", channel_);
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  const auto start_time = Clock::now();
  while (Duration(Clock::now() - start_time).count() <
         wait_for_message_on_initialization_timeout_) {
    // Since the DrakeLcmInterface will not be handling subscriptions during
    // this initialization, we must handle them directly here.
    lcm_->HandleSubscriptions(1 /* timeout_millis*/);
    result = ProcessMessageAndStoreToAbstractState(context, state);
    if (result.severity() != EventStatus::kDidNothing) {
      log()->info("Received message on {}", channel_);
      return result;
    }
  }

  // We ran out of time.
  result = EventStatus::Failed(
      this,
      fmt::format(
          "Timed out without receiving any message on channel {} at url {}",
          channel_, lcm_->get_lcm_url()));
  // TODO(russt): Once EventStatus are actually propagated, return the status
  // instead of throwing it.
  throw std::runtime_error(result.message());
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake


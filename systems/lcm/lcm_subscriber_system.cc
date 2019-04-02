#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <functional>
#include <iostream>
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

// TODO(jwnimmer-tri) The "serializer xor translator" disjoint implementations
// within the method bodies below are not ideal, because of the code smell, and
// because it is likely confusing for users.  We should take further steps to
// make the Value<LcmMessage> port the primary output port, and find a better
// phrasing for the vector-valued output port for users.  For now though, this
// implementation serves as a transition point where we don't have to rewrite
// the old code yet, but still can supply the AbstractValue port for new code.

// TODO(siyuan): Probably should only have AbstractValue output, and
// change / move the translator for vector base values to a separate translator
// block.
LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel, const LcmAndVectorBaseTranslator* translator,
    std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm,
    int fixed_encoded_size)
    : channel_(channel),
      translator_(translator),
      serializer_(std::move(serializer)),
      fixed_encoded_size_(fixed_encoded_size),
      magic_number_{kMagic} {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_ != nullptr));
  DRAKE_DEMAND(lcm);

  subscription_ = lcm->Subscribe(
      channel_, [this](const void* buffer, int size) {
        this->HandleMessage(buffer, size);
      });
  if (subscription_) {
    subscription_->set_unsubscribe_on_delete(true);
  }

  // Declare the single output port.
  if (translator_ != nullptr) {
    // Invoke the translator allocate method once to provide a model value.
    DeclareVectorOutputPort(*AllocateTranslatorOutputValue(),
                            &LcmSubscriberSystem::CalcTranslatorOutputValue);
  } else {
    // Use the "advanced" method to construct explicit non-member functors
    // to deal with the unusual methods we have available.
    DeclareAbstractOutputPort(
        [this]() {
          return this->AllocateSerializerOutputValue();
        },
        [this](const Context<double>& context, AbstractValue* out) {
          this->CalcSerializerOutputValue(context, out);
        });
  }

  // Declare our two states (message_value, message_count).
  if (is_abstract_state()) {
    static_assert(kStateIndexMessage == 0, "");
    this->DeclareAbstractState(AllocateSerializerOutputValue());
    static_assert(kStateIndexMessageCount == 1, "");
    this->DeclareAbstractState(AbstractValue::Make<int>(0));
  } else {
    DRAKE_DEMAND(is_discrete_state());
    static_assert(kStateIndexMessage == 0, "");
    if (translator_) {
      this->DeclareDiscreteState(*AllocateTranslatorOutputValue());
    } else {
      this->DeclareDiscreteState(1 + fixed_encoded_size_);
    }
    static_assert(kStateIndexMessageCount == 1, "");
    this->DeclareDiscreteState(1 /* size */);
  }

  set_name(make_name(channel_));
}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel, std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, nullptr, std::move(serializer), lcm) {}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel, const LcmAndVectorBaseTranslator& translator,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, &translator, nullptr, lcm) {}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, translator_dictionary.GetTranslator(channel),
                          lcm) {}
#pragma GCC diagnostic pop

LcmSubscriberSystem::~LcmSubscriberSystem() {
  // Violate our class invariant, to help catch use-after-free.
  magic_number_ = 0;
}

void LcmSubscriberSystem::CopyLatestMessageInto(State<double>* state) const {
  if (is_discrete_state()) {
    ProcessMessageAndStoreToDiscreteState(&state->get_mutable_discrete_state());
  } else {
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }
}

// The decision to store the decoded message instead of the raw bytes in
// state is to avoid repeated decoding in DoCalcOutput for the latter case.
// However, this computational concern will not exist once caching is properly
// implemented and in place. Same for ProcessMessageAndStoreToAbstractState()
void LcmSubscriberSystem::ProcessMessageAndStoreToDiscreteState(
    DiscreteValues<double>* discrete_state) const {
  DRAKE_ASSERT(is_discrete_state());

  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    if (translator_) {
      translator_->Deserialize(
          received_message_.data(), received_message_.size(),
          &discrete_state->get_mutable_vector(kStateIndexMessage));
    } else {
      const int received_size = static_cast<int>(received_message_.size());
      if (received_size > fixed_encoded_size_) {
        throw std::runtime_error(fmt::format(
            "LcmSubscriberSystem: Received {} message was {} bytes, not the "
            "at-most-{} bytes that was promised to our constructor", channel_,
            received_size, fixed_encoded_size_));
      }
      auto& xd = discrete_state->get_mutable_vector(kStateIndexMessage);
      xd[0] = received_size;
      for (int i = 0; i < received_size; ++i) {
        xd[i + 1] = received_message_[i];
      }
    }
  }
  discrete_state->get_mutable_vector(kStateIndexMessageCount)
      .SetAtIndex(0, received_message_count_);
}

void LcmSubscriberSystem::ProcessMessageAndStoreToAbstractState(
    AbstractValues* abstract_state) const {
  DRAKE_ASSERT(is_abstract_state());
  DRAKE_ASSERT(serializer_ != nullptr);

  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    serializer_->Deserialize(
        received_message_.data(), received_message_.size(),
        &abstract_state->get_mutable_value(kStateIndexMessage));
  }
  abstract_state->get_mutable_value(kStateIndexMessageCount)
      .get_mutable_value<int>() = received_message_count_;
}

int LcmSubscriberSystem::GetMessageCount(const Context<double>& context) const {
  // Gets the last message count from either abstract state or discrete state.
  int last_message_count;
  if (is_abstract_state()) {
    last_message_count =
        context.get_abstract_state<int>(kStateIndexMessageCount);
  } else {
    last_message_count = static_cast<int>(
        context.get_discrete_state(kStateIndexMessageCount).GetAtIndex(0));
  }
  return last_message_count;
}

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

  // Schedule an update event at the current time.
  *time = context.get_time();
  if (is_abstract_state()) {
    EventCollection<UnrestrictedUpdateEvent<double>>& uu_events =
        events->get_mutable_unrestricted_update_events();
    uu_events.add_event(
        std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
            TriggerType::kTimed));
  } else {
    EventCollection<DiscreteUpdateEvent<double>>& du_events =
        events->get_mutable_discrete_update_events();
    du_events.add_event(
        std::make_unique<systems::DiscreteUpdateEvent<double>>(
            TriggerType::kTimed));
  }
}

std::string LcmSubscriberSystem::make_name(const std::string& channel) {
  return "LcmSubscriberSystem(" + channel + ")";
}

const std::string& LcmSubscriberSystem::get_channel_name() const {
  return channel_;
}

// This is only called if our output port is vector-valued, because we are
// using a translator.
std::unique_ptr<BasicVector<double>>
LcmSubscriberSystem::AllocateTranslatorOutputValue() const {
  DRAKE_DEMAND(translator_ != nullptr);
  auto result = translator_->AllocateOutputVector();
  if (result) {
    return result;
  }
  return std::make_unique<BasicVector<double>>(translator_->get_vector_size());
}

void LcmSubscriberSystem::CalcTranslatorOutputValue(
    const Context<double>& context, BasicVector<double>* output_vector) const {
  DRAKE_DEMAND(is_discrete_state());
  output_vector->SetFrom(context.get_discrete_state(kStateIndexMessage));
}

// This is only called if our output port is abstract-valued, because we are
// using a serializer.
std::unique_ptr<AbstractValue>
LcmSubscriberSystem::AllocateSerializerOutputValue() const {
  DRAKE_DEMAND(serializer_ != nullptr);
  return serializer_->CreateDefaultValue();
}

void LcmSubscriberSystem::CalcSerializerOutputValue(
    const Context<double>& context, AbstractValue* output_value) const {
  DRAKE_DEMAND(serializer_ != nullptr);
  if (is_abstract_state()) {
    output_value->SetFrom(
        context.get_abstract_state().get_value(kStateIndexMessage));
  } else {
    if (GetMessageCount(context) > 0) {
      const auto& xd = context.get_discrete_state(kStateIndexMessage);
      const int size = xd[0];
      std::vector<uint8_t> buffer;
      buffer.resize(size);
      for (int i = 0; i < size; ++i) {
        buffer[i] = xd[i + 1];
      }
      serializer_->Deserialize(buffer.data(), size, output_value);
    }
  }
}

void LcmSubscriberSystem::HandleMessage(const void* buffer, int size) {
  SPDLOG_TRACE(drake::log(), "Receiving LCM {} message", channel_);
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
  DRAKE_ASSERT(serializer_ != nullptr);

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

const LcmAndVectorBaseTranslator& LcmSubscriberSystem::get_translator() const {
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake


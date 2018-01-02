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
    drake::lcm::DrakeLcmInterface* lcm)
    : channel_(channel),
      translator_(translator),
      serializer_(std::move(serializer)),
      lcm_interface_(lcm) {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_ != nullptr));
  DRAKE_DEMAND(lcm);

  lcm->Subscribe(channel_, this);

  if (translator_ != nullptr) {
    // Invoke the translator allocate method once to provide a model value.
    DeclareVectorOutputPort(*AllocateTranslatorOutputValue(),
                            &LcmSubscriberSystem::CalcTranslatorOutputValue);
  } else {
    // Use the "advanced" method to construct explicit non-member functors
    // to deal with the unusual methods we have available.
    DeclareAbstractOutputPort(
        [this](const Context<double>&) {
          return this->AllocateSerializerOutputValue();
        },
        [this](const Context<double>& context, AbstractValue* out) {
          this->CalcSerializerOutputValue(context, out);
        });
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

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, translator_dictionary.GetTranslator(channel),
                          lcm) {}

LcmSubscriberSystem::~LcmSubscriberSystem() {}

void LcmSubscriberSystem::SetDefaultState(const Context<double>&,
                                          State<double>* state) const {
  if (translator_ != nullptr) {
    DRAKE_DEMAND(serializer_ == nullptr);
    ProcessMessageAndStoreToDiscreteState(&state->get_mutable_discrete_state());
  } else {
    DRAKE_DEMAND(translator_ == nullptr);
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }
}

// The decision to store the decoded message instead of the raw bytes in
// state is to avoid repeated decoding in DoCalcOutput for the latter case.
// However, this computational concern will not exist once caching is properly
// implemented and in place. Same for ProcessMessageAndStoreToAbstractState()
void LcmSubscriberSystem::ProcessMessageAndStoreToDiscreteState(
    DiscreteValues<double>* discrete_state) const {
  DRAKE_ASSERT(translator_ != nullptr);
  DRAKE_ASSERT(serializer_ == nullptr);

  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    translator_->Deserialize(
        received_message_.data(), received_message_.size(),
        &discrete_state->get_mutable_vector(kStateIndexMessage));
  }
  discrete_state->get_mutable_vector(kStateIndexMessageCount)
      .SetAtIndex(0, received_message_count_);
}

void LcmSubscriberSystem::ProcessMessageAndStoreToAbstractState(
    AbstractValues* abstract_state) const {
  DRAKE_ASSERT(translator_ == nullptr);
  DRAKE_ASSERT(serializer_ != nullptr);

  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    serializer_->Deserialize(
        received_message_.data(), received_message_.size(),
        &abstract_state->get_mutable_value(kStateIndexMessage));
  }
  abstract_state->get_mutable_value(kStateIndexMessageCount)
      .GetMutableValue<int>() = received_message_count_;
}

int LcmSubscriberSystem::GetMessageCount(const Context<double>& context) const {
  // Gets the last message count from either abstract state or discrete state.
  int last_message_count;
  if (translator_ == nullptr) {
    DRAKE_ASSERT(serializer_ != nullptr);
    last_message_count =
        context.get_abstract_state<int>(kStateIndexMessageCount);
  } else {
    DRAKE_ASSERT(serializer_ == nullptr);
    last_message_count = static_cast<int>(
        context.get_discrete_state(kStateIndexMessageCount).GetAtIndex(0));
  }
  return last_message_count;
}

void LcmSubscriberSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
  const int last_message_count = GetMessageCount(context);

  const int received_message_count = [this]() {
    std::unique_lock<std::mutex> lock(received_message_mutex_);
    return received_message_count_;
  }();

  // Has a new message. Schedule an update event.
  if (last_message_count != received_message_count) {
    // TODO(siyuan): should be context.get_time() once #5725 is resolved.
    *time = context.get_time() + 0.0001;
    if (translator_ == nullptr) {
      EventCollection<UnrestrictedUpdateEvent<double>>& uu_events =
          events->get_mutable_unrestricted_update_events();
      uu_events.add_event(
          std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
              Event<double>::TriggerType::kTimed));
    } else {
      EventCollection<DiscreteUpdateEvent<double>>& du_events =
          events->get_mutable_discrete_update_events();
      du_events.add_event(
          std::make_unique<systems::DiscreteUpdateEvent<double>>(
              Event<double>::TriggerType::kTimed));
    }
  } else {
    // Special code to support LCM log playback. For the normal and mock LCM
    // interfaces, this always returns inf and returns.
    *time = lcm_interface_->GetNextMessageTime();
    DRAKE_DEMAND(*time > context.get_time());
    if (std::isinf(*time)) {
      return;
    }

    // Schedule a publish event at the next message time. We use a publish
    // event here because we only want to generate a side effect to dispatch
    // the message properly to all the subscribers, not mutating this system's
    // context.)
    // Note that for every LCM subscriber in the diagram, they will all schedule
    // this event for themselves. However, only the first subscriber executing
    // the callback will advance the log and do the message dispatch. This is
    // because once the first callback is executed, the front of the log will
    // have a different timestamp than the context's time (scheduled callback
    // time).
    EventCollection<PublishEvent<double>>& pub_events =
        events->get_mutable_publish_events();

    PublishEvent<double>::PublishCallback callback = [this](
        const Context<double>& c, const PublishEvent<double>&) {
      // Want to keep polling from the message queue, if they happen
      // to be occur at the exact same time.
      while (lcm_interface_->GetNextMessageTime() == c.get_time()) {
        lcm_interface_->DispatchMessageAndAdvanceLog(c.get_time());
      }
    };

    pub_events.add_event(std::make_unique<systems::PublishEvent<double>>(
        Event<double>::TriggerType::kTimed, callback));
  }
}

std::unique_ptr<DiscreteValues<double>>
LcmSubscriberSystem::AllocateDiscreteState() const {
  // Only make discrete states if we are outputting vector values.
  if (translator_ != nullptr) {
    DRAKE_DEMAND(serializer_ == nullptr);
    std::vector<std::unique_ptr<BasicVector<double>>> discrete_state_vec(2);
    discrete_state_vec[kStateIndexMessage] =
        this->LcmSubscriberSystem::AllocateTranslatorOutputValue();
    discrete_state_vec[kStateIndexMessageCount] =
        std::make_unique<BasicVector<double>>(1);
    return std::make_unique<DiscreteValues<double>>(
        std::move(discrete_state_vec));
  }
  DRAKE_DEMAND(serializer_ != nullptr);
  return LeafSystem<double>::AllocateDiscreteState();
}

std::unique_ptr<AbstractValues> LcmSubscriberSystem::AllocateAbstractState()
    const {
  // Only make abstract states if we are outputting abstract message.
  if (serializer_ != nullptr) {
    DRAKE_DEMAND(translator_ == nullptr);
    std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
    abstract_vals[kStateIndexMessage] =
        this->LcmSubscriberSystem::AllocateSerializerOutputValue();
    abstract_vals[kStateIndexMessageCount] = AbstractValue::Make<int>(0);
    return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
  }
  DRAKE_DEMAND(translator_ != nullptr);
  return LeafSystem<double>::AllocateAbstractState();
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
  DRAKE_DEMAND(translator_ != nullptr && serializer_ == nullptr);
  auto result = translator_->AllocateOutputVector();
  if (result) {
    return result;
  }
  return std::make_unique<BasicVector<double>>(translator_->get_vector_size());
}

void LcmSubscriberSystem::CalcTranslatorOutputValue(
    const Context<double>& context, BasicVector<double>* output_vector) const {
  DRAKE_DEMAND(translator_ != nullptr && serializer_ == nullptr);
  output_vector->SetFrom(context.get_discrete_state(kStateIndexMessage));
}

// This is only called if our output port is abstract-valued, because we are
// using a serializer.
std::unique_ptr<systems::AbstractValue>
LcmSubscriberSystem::AllocateSerializerOutputValue() const {
  DRAKE_DEMAND(translator_ == nullptr && serializer_ != nullptr);
  return serializer_->CreateDefaultValue();
}

void LcmSubscriberSystem::CalcSerializerOutputValue(
    const Context<double>& context, AbstractValue* output_value) const {
  DRAKE_DEMAND(serializer_.get() != nullptr);
  output_value->SetFrom(
      context.get_abstract_state().get_value(kStateIndexMessage));
}

void LcmSubscriberSystem::HandleMessage(const std::string& channel,
                                        const void* message_buffer,
                                        int message_size) {
  SPDLOG_TRACE(drake::log(), "Receiving LCM {} message", channel);

  if (channel == channel_) {
    const uint8_t* const rbuf_begin =
        static_cast<const uint8_t*>(message_buffer);
    const uint8_t* const rbuf_end = rbuf_begin + message_size;
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    received_message_.clear();
    received_message_.insert(received_message_.begin(), rbuf_begin, rbuf_end);

    received_message_count_++;
    received_message_condition_variable_.notify_all();
  } else {
    std::cerr << "LcmSubscriberSystem: HandleMessage: WARNING: Received a "
              << "message for channel \"" << channel
              << "\" instead of channel \"" << channel_ << "\". Ignoring it."
              << std::endl;
  }
}

int LcmSubscriberSystem::WaitForMessage(int old_message_count) const {
  // The message buffer and counter are updated in HandleMessage(), which is
  // a callback function invoked by a different thread owned by the
  // drake::lcm::DrakeLcmInterface instance passed to the constructor. Thus,
  // for thread safety, these need to be properly protected by a mutex.
  std::unique_lock<std::mutex> lock(received_message_mutex_);

  // This while loop is necessary to guard for spurious wakeup:
  // https://en.wikipedia.org/wiki/Spurious_wakeup
  while (old_message_count == received_message_count_)
    // When wait returns, lock is atomically acquired. So it's thread safe to
    // read received_message_count_.
    received_message_condition_variable_.wait(lock);
  int new_message_count = received_message_count_;
  lock.unlock();

  return new_message_count;
}

const LcmAndVectorBaseTranslator& LcmSubscriberSystem::get_translator() const {
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

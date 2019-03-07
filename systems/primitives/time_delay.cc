#include "drake/systems/primitives/time_delay.h"

#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
class DelayBuffer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DelayBuffer)

  DelayBuffer(int buffer_size, std::unique_ptr<BasicVector<T>> initial_value)
      : index_(0), buffer_size_(buffer_size) {
    buffer_.push_back(std::move(initial_value));
    for (int ii = 1; ii < buffer_size_; ii++) {
      buffer_.push_back(buffer_[0]->Clone());
    }
  }

  void UpdateBuffer(const Eigen::Ref<const VectorX<T>>& new_value) {
    buffer_[index_ % buffer_size_]->SetFromVector(new_value);
    index_++;
  }

  const Eigen::VectorBlock<const VectorX<T>> GetDelayedValue() const {
    return buffer_[index_ % buffer_size_]->get_value();
  }

  std::unique_ptr<DelayBuffer<T>> Clone() const {
    auto clone =
        std::make_unique<DelayBuffer<T>>(buffer_size_, buffer_[0]->Clone());
    clone->index_ = index_;
    for (int ii = 1; ii < buffer_size_; ii++) {
      clone->buffer_[ii]->SetFrom(*buffer_[ii]);
    }
    return clone;
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class DelayBuffer;

  int index_{};
  const int buffer_size_{};
  std::vector<std::unique_ptr<BasicVector<T>>> buffer_;
};

}  // anonymous namespace

template <typename T>
TimeDelay<T>::TimeDelay(double delay_sec, double update_sec, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::TimeDelay>()),
      delay_sec_(delay_sec),
      update_sec_(update_sec),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    BasicVector<T> model_value(VectorX<T>::Zero(vector_size));
    this->DeclareVectorInputPort(model_value);
    this->DeclareVectorOutputPort(model_value, &TimeDelay::DoCalcVectorOutput);
    this->DeclareAbstractState(AbstractValue::Make(
        DelayBuffer<T>(delay_sec_/update_sec_, model_value.Clone())));
    this->DeclarePeriodicUnrestrictedUpdate(update_sec_);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    DRAKE_ASSERT(!is_abstract());  // AbstractValue not yet supported
    // this->DeclareAbstractInputPort(*abstract_model_value_);
    // this->DeclareAbstractOutputPort(
    //     [this]() { return abstract_model_value_->Clone(); },
    //     [this](const Context<double>& context, AbstractValue* out) {
    //       this->DoCalcAbstractOutput(context, out);
    //     });
    // this->DeclareAbstractState(
    //     AbstractValue::Make(DelayBuffer<const AbstractValue>(
    //         delay_sec_ / update_sec_, abstract_model_value_->Clone())));
    // this->DeclarePeriodicUnrestrictedUpdate(update_sec_);
  }
}

template <typename T>
template <typename U>
TimeDelay<T>::TimeDelay(const TimeDelay<U>& other)
    : TimeDelay(other.delay_sec_, other.update_sec_,
          other.is_abstract() ? -1 : other.get_input_port().size(),
          other.is_abstract() ? other.abstract_model_value_->Clone()
                              : nullptr) {}

template <typename T>
void TimeDelay<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const DelayBuffer<T>& delay_buffer =
      context.template get_abstract_state<DelayBuffer<T>>(0);
  output->SetFromVector(delay_buffer.GetDelayedValue());
}

template <typename T>
void TimeDelay<T>::DoCalcAbstractOutput(const Context<T>&,
                                            AbstractValue*) const {
  DRAKE_ASSERT(is_abstract());
  DRAKE_ASSERT(!is_abstract());  // AbstractValue not yet supported
  // const DelayBuffer<const AbstractValue> delay_buffer =
  //     context.get_abstract_state<DelayBuffer<const AbstractValue>>(0);
  // output->SetFrom(delay_buffer.GetDelayedValue());
}

template <typename T>
void TimeDelay<T>::DoCalcUnrestrictedUpdate(
    const Context<T>& context,
    const std::vector<const UnrestrictedUpdateEvent<T>*>&,
    State<T>* state) const {
  if (!is_abstract()) {
    const auto& input = get_input_port().Eval(context);
    DelayBuffer<T>& delay_buffer =
        state->template get_mutable_abstract_state<DelayBuffer<T>>(0);
    delay_buffer.UpdateBuffer(input);
  } else {
    DRAKE_ASSERT(!is_abstract());  // AbstractValue not yet supported
    // const auto& input = get_input_port().Eval(context);  // NOLINT
    // DelayBuffer<const AbstractValue>& delay_buffer =
    //     state->get_mutable_abstract_state<DelayBuffer<const AbstractValue>>(0);  // NOLINT
    // delay_buffer.UpdateBuffer(input);
  }
}

template <typename T>
optional<bool> TimeDelay<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a time delay will not have direct feedthrough, as the
  // output only depends on the state, not the input.
  return false;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TimeDelay)

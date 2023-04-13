#include "drake/systems/primitives/delay_line.h"

#include <deque>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"

namespace drake {
namespace systems {
namespace {

struct DelayLineState {
  DelayLineState(int num_samples, const AbstractValue& sample) {
    samples.resize(num_samples, copyable_unique_ptr<AbstractValue>(sample));
  }

  std::deque<copyable_unique_ptr<AbstractValue>> samples;
};

}  // namespace

template <typename T>
DelayLine<T>::DelayLine(int num_samples, double period,
                        std::optional<int> vector_size,
                        std::unique_ptr<const AbstractValue> model_value)
    : LeafSystem<T>(SystemTypeTag<DelayLine>()),
      num_samples_(num_samples),
      period_(period),
      model_value_(std::move(model_value)) {
  DRAKE_THROW_UNLESS(num_samples >= 1);
  DRAKE_THROW_UNLESS(period > 0.0);
  this->DeclarePeriodicUnrestrictedUpdateEvent(period, 0.0, &DelayLine::Update);
  if (is_abstract()) {
    DRAKE_DEMAND(!vector_size.has_value());
    DRAKE_DEMAND(model_value_ != nullptr);
    this->DeclareAbstractInputPort("u", *model_value_);
    this->DeclareAbstractState(
        Value<DelayLineState>(num_samples, *model_value_));
    this->DeclareAbstractOutputPort(
        "y",
        [this]() {
          return model_value_->Clone();
        },
        [](const Context<T>& context, AbstractValue* y) {
          DRAKE_DEMAND(y != nullptr);
          const DelayLineState& buffer =
              context.template get_abstract_state<DelayLineState>(0);
          y->SetFrom(*buffer.samples.front());
        },
        {this->all_state_ticket()});
  } else {
    DRAKE_DEMAND(vector_size.has_value());
    DRAKE_DEMAND(model_value_ == nullptr);
    this->DeclareVectorInputPort("u", *vector_size);
    this->DeclareAbstractState(Value<DelayLineState>(
        num_samples,
        Value<BasicVector<double>>(Eigen::VectorXd::Zero(*vector_size))));
    this->DeclareVectorOutputPort(
        "u", *vector_size,
        [](const Context<T>& context, BasicVector<T>* y) {
          DRAKE_DEMAND(y != nullptr);
          const DelayLineState& buffer =
              context.template get_abstract_state<DelayLineState>(0);
          y->SetFrom(buffer.samples.front()->get_value<BasicVector<T>>());
        },
        {this->all_state_ticket()});
  }
}

template <typename T>
DelayLine<T>::DelayLine(int num_samples, double period, int vector_size)
    : DelayLine<T>(num_samples, period, vector_size, {}) {}

template <typename T>
DelayLine<T>::DelayLine(int num_samples, double period,
                        const AbstractValue& model_value)
    : DelayLine<T>(num_samples, period, {}, model_value.Clone()) {}

template <typename T>
template <typename U>
DelayLine<T>::DelayLine(const DelayLine<U>& other)
    : DelayLine(other.num_samples(), other.period(),
                other.is_abstract()
                    ? std::nullopt
                    : std::optional<int>(other.get_input_port().size()),
                other.is_abstract()  // BR
                    ? other.model_value_->Clone()
                    : nullptr) {}

template <typename T>
DelayLine<T>::~DelayLine() = default;

template <typename T>
void DelayLine<T>::Update(const Context<T>& context, State<T>* state) const {
  const AbstractValue& input =
      this->get_input_port().template Eval<AbstractValue>(context);
  DelayLineState& buffer =
      state->template get_mutable_abstract_state<DelayLineState>(0);
  buffer.samples.pop_front();
  buffer.samples.emplace_back(input);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DelayLine)

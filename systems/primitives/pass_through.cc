#include "drake/systems/primitives/pass_through.h"

#include <utility>

namespace drake {
namespace systems {

template <typename T>
PassThrough<T>::PassThrough(
    const Eigen::Ref<const Eigen::VectorXd>& model_vector,
    std::unique_ptr<const AbstractValue> abstract_model_value,
    bool input_required)
    : LeafSystem<T>(SystemTypeTag<PassThrough>()),
      model_vector_(model_vector),
      abstract_model_value_(std::move(abstract_model_value)),
      input_required_(input_required) {
  if (!is_abstract()) {
    input_port_ =
        &this->DeclareVectorInputPort("u", BasicVector<T>(model_vector));
    this->DeclareVectorOutputPort("y", BasicVector<T>(model_vector),
                                  &PassThrough::DoCalcVectorOutput,
                                  {this->all_input_ports_ticket()});
  } else {
    DRAKE_DEMAND(model_vector.size() == 0);
    input_port_ = &this->DeclareAbstractInputPort("u", *abstract_model_value_);

    namespace sp = std::placeholders;
    this->DeclareAbstractOutputPort(
        "y",
        [this]() {
          return abstract_model_value_->Clone();
        },
        std::bind(&PassThrough::DoCalcAbstractOutput, this, sp::_1, sp::_2),
        {this->all_input_ports_ticket()});
  }
}

template <typename T>
template <typename U>
PassThrough<T>::PassThrough(const PassThrough<U>& other)
    : PassThrough(other.model_vector_,
                  other.is_abstract() ? other.abstract_model_value_->Clone()
                                      : nullptr,
                  other.input_required_) {}

template <typename T>
PassThrough<T>::~PassThrough() = default;

template <typename T>
void PassThrough<T>::DoCalcVectorOutput(const Context<T>& context,
                                        BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  if (input_required_ || this->get_input_port().HasValue(context)) {
    // When input_required_ is true and the port is unconnected, Eval throws.
    const auto& input = this->get_input_port().Eval(context);
    DRAKE_ASSERT(input.size() == output->size());
    output->get_mutable_value() = input;
  } else {
    output->get_mutable_value() = model_vector_;
  }
}

template <typename T>
void PassThrough<T>::DoCalcAbstractOutput(const Context<T>& context,
                                          AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  if (input_required_ || this->get_input_port().HasValue(context)) {
    // When input_required_ is true and the port is unconnected, Eval throws.
    output->SetFrom(
        this->get_input_port().template Eval<AbstractValue>(context));
  } else {
    output->SetFrom(*abstract_model_value_);
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PassThrough);

#include "drake/systems/framework/value_calc_function.h"

namespace drake {
namespace systems {
namespace internal {

AbstractValueCloner::AbstractValueCloner(std::unique_ptr<AbstractValue> model)
    : model_(std::move(model)) {}

AbstractValueCloner::~AbstractValueCloner() = default;

std::unique_ptr<AbstractValue> AbstractValueCloner::operator()() const {
  return model_->Clone();
}

}  // namespace internal

ValueCalcFunction::ValueCalcFunction() = default;
ValueCalcFunction::~ValueCalcFunction() = default;

std::unique_ptr<AbstractValue> ValueCalcFunction::Allocate() const {
  return alloc_();
}

void ValueCalcFunction::Calc(const ContextBase& context,
                             AbstractValue* output) const {
  DRAKE_DEMAND(output != nullptr);
  return calc_(context, output);
}

}  // namespace systems
}  // namespace drake

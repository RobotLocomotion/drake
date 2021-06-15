#include "drake/systems/framework/value_calc_function.h"

#include <stdexcept>

namespace drake {
namespace systems {
namespace internal {

AbstractValueCloner::AbstractValueCloner(std::unique_ptr<AbstractValue> model)
    : model_(std::move(model)) {
  DRAKE_DEMAND(model_ != nullptr);
}

AbstractValueCloner::~AbstractValueCloner() = default;

std::unique_ptr<AbstractValue> AbstractValueCloner::operator()() const {
  return model_->Clone();
}

}  // namespace internal

ValueCalcFunction::ValueCalcFunction() = default;

ValueCalcFunction::ValueCalcFunction(
    AllocateCallback allocate, CalcCallback calc)
    : allocate_(std::move(allocate)), calc_(std::move(calc)) {
  if (allocate_ == nullptr) {
    throw std::logic_error(
        "Cannot create a ValueCalcFunction with a null AllocateCallback");
  }
  if (calc_ == nullptr) {
    throw std::logic_error(
        "Cannot create a ValueCalcFunction with a null Calc");
  }
}

ValueCalcFunction::~ValueCalcFunction() = default;

void ValueCalcFunction::NoopCalc(const ContextBase&, AbstractValue*) {}

std::unique_ptr<AbstractValue> ValueCalcFunction::Allocate() const {
  if (allocate_ == nullptr) {
    throw std::logic_error(
        "ValueCalcFunction cannot invoke a null AllocateCallback");
  }
  return allocate_();
}

void ValueCalcFunction::Calc(const ContextBase& context,
                             AbstractValue* output) const {
  DRAKE_DEMAND(output != nullptr);
  if (calc_ == nullptr) {
    throw std::logic_error(
        "ValueCalcFunction cannot invoke a null CalcCallback");
  }
  return calc_(context, output);
}

}  // namespace systems
}  // namespace drake

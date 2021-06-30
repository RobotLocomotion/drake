#include "drake/systems/framework/value_producer.h"

#include <stdexcept>
#include <utility>

namespace drake {
namespace systems {

ValueProducer::ValueProducer() = default;

ValueProducer::ValueProducer(
    AllocateCallback allocate, CalcCallback calc)
    : allocate_(std::move(allocate)), calc_(std::move(calc)) {
  if (allocate_ == nullptr) {
    throw std::logic_error(
        "Cannot create a ValueProducer with a null AllocateCallback");
  }
  if (calc_ == nullptr) {
    throw std::logic_error(
        "Cannot create a ValueProducer with a null Calc");
  }
}

ValueProducer::~ValueProducer() = default;

bool ValueProducer::is_valid() const {
  return (allocate_ != nullptr) && (calc_ != nullptr);
}

void ValueProducer::NoopCalc(const ContextBase&, AbstractValue*) {}

std::unique_ptr<AbstractValue> ValueProducer::Allocate() const {
  if (allocate_ == nullptr) {
    throw std::logic_error(
        "ValueProducer cannot invoke a null AllocateCallback");
  }
  return allocate_();
}

void ValueProducer::Calc(const ContextBase& context,
                         AbstractValue* output) const {
  if (output == nullptr) {
    throw std::logic_error(
        "ValueProducer output was nullptr");
  }
  if (calc_ == nullptr) {
    throw std::logic_error(
        "ValueProducer cannot invoke a null CalcCallback");
  }
  return calc_(context, output);
}

}  // namespace systems
}  // namespace drake

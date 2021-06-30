#include "drake/systems/framework/abstract_value_cloner.h"

#include <utility>

namespace drake {
namespace systems {
namespace internal {

// We don't need to clone the model_value here (we can move it directly into
// our storage) because this constructor is private and the public constructor
// always makes a copy before calling us, which means we're guaranteed that
// nobody else has an alias of this model_value to mutate it out from under us.
AbstractValueCloner::AbstractValueCloner(
    std::unique_ptr<AbstractValue> model_value)
    : model_value_(std::move(model_value)) {
  DRAKE_DEMAND(model_value_ != nullptr);
}

AbstractValueCloner::~AbstractValueCloner() = default;

std::unique_ptr<AbstractValue> AbstractValueCloner::operator()() const {
  return model_value_->Clone();
}

}  // namespace internal
}  // namespace systems
}  // namespace drake

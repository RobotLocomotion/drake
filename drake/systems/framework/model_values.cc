#include "drake/systems/framework/model_values.h"

#include <memory>
#include <vector>

namespace drake {
namespace systems {
namespace detail {

int ModelValues::size() const {
  return static_cast<int>(values_.size());
}

void ModelValues::AddModel(
    int index, std::unique_ptr<AbstractValue> model_value) {
  // Grow the values_ so that our new model will live at @p index.
  DRAKE_DEMAND(index >= size());
  values_.resize(index);
  values_.emplace_back(std::move(model_value));
}

std::unique_ptr<AbstractValue> ModelValues::CloneModel(int index) const {
  if (index < size()) {
    const AbstractValue* const model_value = values_[index].get();
    if (model_value != nullptr) {
      std::unique_ptr<AbstractValue> result = model_value->Clone();
      DRAKE_DEMAND(result.get() != nullptr);
      return result;
    }
  }
  return nullptr;
}


}  // namespace detail
}  // namespace systems
}  // namespace drake

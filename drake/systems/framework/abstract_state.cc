#include "drake/systems/framework/abstract_state.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

AbstractState::~AbstractState() {}

AbstractState::AbstractState() {}

AbstractState::AbstractState(std::vector<std::unique_ptr<AbstractValue>>&& data)
    : owned_data_(std::move(data)) {
  for (auto& datum : owned_data_) {
    data_.push_back(datum.get());
  }
}

AbstractState::AbstractState(const std::vector<AbstractValue*>& data)
      : data_(data) {}

int AbstractState::size() const {
  return static_cast<int>(data_.size());
}

const AbstractValue& AbstractState::get_abstract_state(int index) const {
  DRAKE_ASSERT(index >= 0 && index < size());
  DRAKE_ASSERT(data_[index] != nullptr);
  return *data_[index];
}

AbstractValue& AbstractState::get_mutable_abstract_state(int index) {
  DRAKE_ASSERT(index >= 0 && index < size());
  DRAKE_ASSERT(data_[index] != nullptr);
  return *data_[index];
}

void AbstractState::CopyFrom(const AbstractState& other) {
  DRAKE_ASSERT(size() == other.size());
  for (int i = 0; i < size(); i++) {
    DRAKE_ASSERT(data_[i] != nullptr);
    data_[i]->SetFrom(other.get_abstract_state(i));
  }
}

std::unique_ptr<AbstractState> AbstractState::Clone() const {
  std::vector<std::unique_ptr<AbstractValue>> cloned_data;
  cloned_data.reserve(data_.size());
  for (const AbstractValue* datum : data_) {
    cloned_data.push_back(datum->Clone());
  }
  return std::make_unique<AbstractState>(std::move(cloned_data));
}

}  // namespace systems
}  // namespace drake

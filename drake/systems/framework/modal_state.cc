#include "drake/systems/framework/modal_state.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

ModalState::~ModalState() {}

ModalState::ModalState() {}

ModalState::ModalState(std::vector<std::unique_ptr<AbstractValue>>&& data)
    : owned_data_(std::move(data)) {
  for (auto& datum : owned_data_) {
    data_.push_back(datum.get());
  }
}

ModalState::ModalState(const std::vector<AbstractValue*>& data)
      : data_(data) {}

int ModalState::size() const {
  return static_cast<int>(data_.size());
}

const AbstractValue& ModalState::get_modal_state(int index) const {
  DRAKE_ASSERT(index >= 0 && index < size());
  DRAKE_ASSERT(data_[index] != nullptr);
  return *data_[index];
}

AbstractValue& ModalState::get_mutable_modal_state(int index) {
  DRAKE_ASSERT(index >= 0 && index < size());
  DRAKE_ASSERT(data_[index] != nullptr);
  return *data_[index];
}

void ModalState::CopyFrom(const ModalState& other) {
  DRAKE_DEMAND(size() == other.size());
  for (int i = 0; i < size(); i++) {
    data_[i]->SetFrom(other.get_modal_state(i));
  }
}

std::unique_ptr<ModalState> ModalState::Clone() const {
  std::vector<std::unique_ptr<AbstractValue>> cloned_data;
  cloned_data.reserve(data_.size());
  for (const AbstractValue* datum : data_) {
    cloned_data.push_back(datum->Clone());
  }
  return std::make_unique<ModalState>(std::move(cloned_data));
}

}  // namespace systems
}  // namespace drake

#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class DiscreteState {
 public:
  DiscreteState(std::vector<std::unique_ptr<BasicVector<T>>> difference_state,
                std::vector<std::unique_ptr<AbstractValue>> modal_state)
      : difference_state_(std::move(difference_state)),
        modal_state_(std::move(modal_state)) {}

  virtual ~DiscreteState() {}

  int get_difference_state_size() const {
    return difference_state_.size();
  }

  const BasicVector<T>* get_difference_state(int index) const {
    DRAKE_ASSERT(index >= 0 && index < difference_state_.size());
    return difference_state_[index].get();
  }

  BasicVector<T>* get_mutable_difference_state(int index) {
    DRAKE_ASSERT(index >= 0 && index < difference_state_.size());
    return difference_state_[index].get();
  }

  int get_modal_state_size() const {
    return modal_state_.size();
  }

  const AbstractValue* get_modal_state(int index) const {
    DRAKE_ASSERT(index >= 0 && index < modal_state_.size());
    return modal_state_[index].get();
  }

  AbstractValue* get_mutable_modal_state(int index) {
    DRAKE_ASSERT(index >= 0 && index < modal_state_.size());
    return modal_state_[index].get();
  }

  void CopyFrom(const DiscreteState<T>& other) {
    const int n = get_difference_state_size();
    DRAKE_DEMAND(n == other.get_difference_state_size());
    for (int i = 0; i < n; i++) {
      difference_state_[i]->set_value(
          other.get_difference_state(i)->get_value());
    }

    // TODO(david-german-tri): Copy mode state too.
  }

  // DiscreteState is not copyable or moveable.
  DiscreteState(const DiscreteState& other) = delete;
  DiscreteState& operator=(const DiscreteState& other) = delete;
  DiscreteState(DiscreteState&& other) = delete;
  DiscreteState& operator=(DiscreteState&& other) = delete;

 private:
  std::vector<std::unique_ptr<BasicVector<T>>> difference_state_;
  std::vector<std::unique_ptr<AbstractValue>> modal_state_;
};

}  // namespace systems
}  // namespace drake

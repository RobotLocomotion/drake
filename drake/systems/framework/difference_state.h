#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The DifferenceState is a container for the numerical state values that are
/// updated discontinuously on time or state based triggers.
///
/// DifferenceState owns its state, and is therefore suitable for leaf Systems
/// but not for Diagrams.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class DifferenceState {
 public:
  DifferenceState() {}

  explicit DifferenceState(
      std::vector<std::unique_ptr<BasicVector<T>>> difference_state)
      : difference_state_(std::move(difference_state)) {}

  virtual ~DifferenceState() {}

  int size() const {
    return static_cast<int>(difference_state_.size());
  }

  const BasicVector<T>* get_difference_state(int index) const {
    DRAKE_ASSERT(index >= 0 && index < size());
    return difference_state_[index].get();
  }

  BasicVector<T>* get_mutable_difference_state(int index) {
    DRAKE_ASSERT(index >= 0 && index < size());
    return difference_state_[index].get();
  }

  void CopyFrom(const DifferenceState<T>& other) {
    DRAKE_DEMAND(size() == other.size());
    for (int i = 0; i < size(); i++) {
      difference_state_[i]->set_value(
          other.get_difference_state(i)->get_value());
    }
  }

  // DifferenceState is not copyable or moveable.
  DifferenceState(const DifferenceState& other) = delete;
  DifferenceState& operator=(const DifferenceState& other) = delete;
  DifferenceState(DifferenceState&& other) = delete;
  DifferenceState& operator=(DifferenceState&& other) = delete;

 private:
  std::vector<std::unique_ptr<BasicVector<T>>> difference_state_;
};

}  // namespace systems
}  // namespace drake

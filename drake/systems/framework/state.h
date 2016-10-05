#pragma once

#include <vector>

#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/difference_state.h"

namespace drake {
namespace systems {

/// The State is a container for all the data comprising the complete state of
/// a particular System at a particular moment. Any field in the State may be
/// empty if it is not applicable to the System in question. A System may not
/// maintain state in any place other than the State object.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class State {
 public:
  State() {}
  virtual ~State() {}

  void set_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    continuous_state_ = std::move(xc);
  }

  const ContinuousState<T>* get_continuous_state() const {
    return continuous_state_.get();
  }

  ContinuousState<T>* get_mutable_continuous_state() {
    return continuous_state_.get();
  }

  void set_difference_state(std::unique_ptr<DifferenceState<T>> xd) {
    difference_state_ = std::move(xd);
  }

  const DifferenceState<T>* get_difference_state() const {
    return difference_state_.get();
  }

  DifferenceState<T>* get_mutable_difference_state() {
    return difference_state_.get();
  }

  // State is not copyable or moveable.
  State(const State& other) = delete;
  State& operator=(const State& other) = delete;
  State(State&& other) = delete;
  State& operator=(State&& other) = delete;

 private:
  std::unique_ptr<ContinuousState<T>> continuous_state_;
  std::unique_ptr<DifferenceState<T>> difference_state_;
};

}  // namespace systems
}  // namespace drake

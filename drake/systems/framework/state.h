#pragma once

#include <vector>

#include "drake/systems/framework/abstract_state.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_state.h"

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
  State()
      : abstract_state_(std::make_unique<AbstractState>()),
        continuous_state_(std::make_unique<ContinuousState<T>>()),
        discrete_state_(std::make_unique<DiscreteState<T>>()) {}
  virtual ~State() {}

  void set_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    DRAKE_DEMAND(xc != nullptr);
    continuous_state_ = std::move(xc);
  }

  const ContinuousState<T>* get_continuous_state() const {
    return continuous_state_.get();
  }

  ContinuousState<T>* get_mutable_continuous_state() {
    return continuous_state_.get();
  }

  void set_discrete_state(std::unique_ptr<DiscreteState<T>> xd) {
    DRAKE_DEMAND(xd != nullptr);
    discrete_state_ = std::move(xd);
  }

  const DiscreteState<T>* get_discrete_state() const {
    return discrete_state_.get();
  }

  DiscreteState<T>* get_mutable_discrete_state() {
    return discrete_state_.get();
  }

  void set_abstract_state(std::unique_ptr<AbstractState> xm) {
    DRAKE_DEMAND(xm != nullptr);
    abstract_state_ = std::move(xm);
  }

  const AbstractState* get_abstract_state() const {
    return abstract_state_.get();
  }

  AbstractState* get_mutable_abstract_state() {
    return abstract_state_.get();
  }

  /// Copies the values from another State of the same scalar type into this
  /// State.
  void CopyFrom(const State<T>& other) {
    continuous_state_->CopyFrom(*other.get_continuous_state());
    discrete_state_->CopyFrom(*other.get_discrete_state());
    abstract_state_->CopyFrom(*other.get_abstract_state());
  }

  /// Initializes this state (regardless of scalar type) from a State<double>.
  /// All scalar types in Drake must support initialization from doubles.
  void SetFrom(const State<double>& other) {
    continuous_state_->SetFrom(*other.get_continuous_state());
    discrete_state_->SetFrom(*other.get_discrete_state());
    abstract_state_->CopyFrom(*other.get_abstract_state());
  }

  // State is not copyable or moveable.
  State(const State& other) = delete;
  State& operator=(const State& other) = delete;
  State(State&& other) = delete;
  State& operator=(State&& other) = delete;

 private:
  std::unique_ptr<AbstractState> abstract_state_;
  std::unique_ptr<ContinuousState<T>> continuous_state_;
  std::unique_ptr<DiscreteState<T>> discrete_state_;
};

}  // namespace systems
}  // namespace drake

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/abstract_values.h"
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
  // State is not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(State)

  State()
      : abstract_state_(std::make_unique<AbstractValues>()),
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

  void set_abstract_state(std::unique_ptr<AbstractValues> xm) {
    DRAKE_DEMAND(xm != nullptr);
    abstract_state_ = std::move(xm);
  }

  const AbstractValues* get_abstract_state() const {
    return abstract_state_.get();
  }

  AbstractValues* get_mutable_abstract_state() { return abstract_state_.get(); }

  /// Returns a const pointer to the abstract component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  template <typename U>
  const U& get_abstract_state(int index) const {
    const AbstractValues* xm = get_abstract_state();
    return xm->get_value(index).GetValue<U>();
  }

  /// Returns a mutable pointer to element @p index of the abstract state.
  /// Asserts if @p index doesn't exist.
  template <typename U>
  U& get_mutable_abstract_state(int index) {
    AbstractValues* xm = get_mutable_abstract_state();
    return xm->get_mutable_value(index).GetMutableValue<U>();
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

 private:
  std::unique_ptr<AbstractValues> abstract_state_;
  std::unique_ptr<ContinuousState<T>> continuous_state_;
  std::unique_ptr<DiscreteState<T>> discrete_state_;
};

}  // namespace systems
}  // namespace drake

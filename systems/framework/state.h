#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"

namespace drake {
namespace systems {

/// %State is a container for all the data comprising the
/// complete state of a particular System at a particular moment. Any field in
/// %State may be empty if it is not applicable to the System in question.
/// A System may not maintain state in any place other than a %State object.
///
/// A %State `x` contains three types of state variables:
///
/// - ContinuousState `xc`
/// - DiscreteState   `xd`
/// - AbstractState   `xa`
///
/// @tparam_default_scalar
template <typename T>
class State {
 public:
  // State is not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(State)

  State()
      : abstract_state_(std::make_unique<AbstractValues>()),
        continuous_state_(std::make_unique<ContinuousState<T>>()),
        discrete_state_(std::make_unique<DiscreteValues<T>>()) {}
  virtual ~State() {}

  void set_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    DRAKE_DEMAND(xc != nullptr);
    continuous_state_ = std::move(xc);
  }

  const ContinuousState<T>& get_continuous_state() const {
    DRAKE_ASSERT(continuous_state_ != nullptr);
    return *continuous_state_.get();
  }

  ContinuousState<T>& get_mutable_continuous_state() {
    DRAKE_ASSERT(continuous_state_ != nullptr);
    return *continuous_state_.get();
  }

  void set_discrete_state(std::unique_ptr<DiscreteValues<T>> xd) {
    DRAKE_DEMAND(xd != nullptr);
    discrete_state_ = std::move(xd);
  }

  const DiscreteValues<T>& get_discrete_state() const {
    DRAKE_ASSERT(discrete_state_ != nullptr);
    return *discrete_state_.get();
  }

  DiscreteValues<T>& get_mutable_discrete_state() {
    DRAKE_ASSERT(discrete_state_ != nullptr);
    return *discrete_state_.get();
  }

  const BasicVector<T>& get_discrete_state(int index) const {
    const DiscreteValues<T>& xd = get_discrete_state();
    return xd.get_vector(index);
  }

  BasicVector<T>& get_mutable_discrete_state(int index) {
    DiscreteValues<T>& xd = get_mutable_discrete_state();
    return xd.get_mutable_vector(index);
  }

  void set_abstract_state(std::unique_ptr<AbstractValues> xa) {
    DRAKE_DEMAND(xa != nullptr);
    abstract_state_ = std::move(xa);
  }

  const AbstractValues& get_abstract_state() const {
    DRAKE_ASSERT(abstract_state_ != nullptr);
    return *abstract_state_.get();
  }

  AbstractValues& get_mutable_abstract_state() {
    DRAKE_ASSERT(abstract_state_ != nullptr);
    return *abstract_state_.get();
  }

  /// Returns a const pointer to the abstract component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  template <typename U>
  const U& get_abstract_state(int index) const {
    const AbstractValues& xa = get_abstract_state();
    return xa.get_value(index).get_value<U>();
  }

  /// Returns a mutable pointer to element @p index of the abstract state.
  /// Asserts if @p index doesn't exist.
  template <typename U>
  U& get_mutable_abstract_state(int index) {
    AbstractValues& xa = get_mutable_abstract_state();
    return xa.get_mutable_value(index).get_mutable_value<U>();
  }

  /// Initializes this state from a State<U>.
  template <typename U>
  void SetFrom(const State<U>& other) {
    continuous_state_->SetFrom(other.get_continuous_state());
    discrete_state_->SetFrom(other.get_discrete_state());
    abstract_state_->SetFrom(other.get_abstract_state());
  }

 private:
  std::unique_ptr<AbstractValues> abstract_state_;
  std::unique_ptr<ContinuousState<T>> continuous_state_;
  std::unique_ptr<DiscreteValues<T>> discrete_state_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::State)

#pragma once

#include <vector>

#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/difference_state.h"
#include "drake/systems/framework/modal_state.h"

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
      : continuous_state_(std::make_unique<ContinuousState<T>>()),
        difference_state_(std::make_unique<DifferenceState<T>>()),
        modal_state_(std::make_unique<ModalState>()) {}
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

  void set_difference_state(std::unique_ptr<DifferenceState<T>> xd) {
    DRAKE_DEMAND(xd != nullptr);
    difference_state_ = std::move(xd);
  }

  const DifferenceState<T>* get_difference_state() const {
    return difference_state_.get();
  }

  DifferenceState<T>* get_mutable_difference_state() {
    return difference_state_.get();
  }

  void set_modal_state(std::unique_ptr<ModalState> xm) {
    DRAKE_DEMAND(xm != nullptr);
    modal_state_ = std::move(xm);
  }

  const ModalState* get_modal_state() const {
    return modal_state_.get();
  }

  ModalState* get_mutable_modal_state() {
    return modal_state_.get();
  }

  // State is not copyable or moveable.
  State(const State& other) = delete;
  State& operator=(const State& other) = delete;
  State(State&& other) = delete;
  State& operator=(State&& other) = delete;

 private:
  std::unique_ptr<ContinuousState<T>> continuous_state_;
  std::unique_ptr<DifferenceState<T>> difference_state_;
  std::unique_ptr<ModalState> modal_state_;
};

}  // namespace systems
}  // namespace drake

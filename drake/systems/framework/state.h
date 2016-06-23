#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/systems/framework/state_subvector.h"
#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The ContinuousState is a container for all the State variables that are
/// unique to continuous Systems, i.e. Systems that satisfy
/// ContinuousSystemInterface and have defined dynamics at all times.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class ContinuousState {
 public:
  /// Constructs a ContinuousState for a system that does not have second-order
  /// structure: All of the state is misc_continuous_state_.
  explicit ContinuousState(std::unique_ptr<StateVector<T>> state) {
    state_ = std::move(state);
    generalized_position_.reset(new StateSubvector<T>(state_.get()));
    generalized_velocity_.reset(new StateSubvector<T>(state_.get()));
    misc_continuous_state_.reset(
        new StateSubvector<T>(state_.get(), 0, state_.size()));
  }

  /// Constructs a ContinuousState that exposes second-order structure.
  /// The contents of @p state must be laid out as follows:
  ///
  /// @verbatim
  /// (index 0)|--q--|--v--|--z--|(index state.size() - 1)
  ///
  /// Where q is generalized position
  ///       v is generalized velocity
  ///       z is other continuous state
  /// @endverbatim
  ///
  /// @param num_q The number of position variables.
  /// @param num_v The number of velocity variables.
  /// @param num_z  The number of other variables.
  ContinuousState(std::unique_ptr<StateVector<T>> state, int64_t num_q,
                  int64_t num_v, int64_t num_z) {
    state_ = std::move(state);
    if (state_->size() != num_q + num_v + num_z) {
      throw std::out_of_range(
          "Continuous state of size " + std::to_string(state_->size()) +
          "cannot be partitioned as" + " q " + std::to_string(num_q) + " v " +
          std::to_string(num_v) + " z " + std::to_string(num_z));
    }
    if (num_v > num_q) {
      throw std::logic_error("Number of velocity variables " +
                             std::to_string(num_v) +
                             " must not exceed number of position variables " +
                             std::to_string(num_q));
    }
    generalized_position_.reset(new StateSubvector<T>(state_.get(), 0, num_q));
    generalized_velocity_.reset(
        new StateSubvector<T>(state_.get(), num_q, num_v));
    misc_continuous_state_.reset(
        new StateSubvector<T>(state_.get(), num_q + num_v, num_z));
  }

  // TODO(david-german-tri): Add a suitable constructor for the continuous
  // state of a Diagram, using StateSupervectors.

  /// Returns the entire state vector.
  const StateVector<T>& get_state() const { return *state_; }

  /// Returns a mutable pointer to the entire state vector, never null.
  StateVector<T>* get_mutable_state() { return state_.get(); }

  /// Returns the subset of the state vector that is generalized position `q`.
  const StateVector<T>& get_generalized_position() const {
    return *generalized_position_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// generalized position `q`.
  StateVector<T>* get_mutable_generalized_position() {
    return generalized_position_.get();
  }

  /// Returns the subset of the state vector that is generalized velocity `v`.
  const StateVector<T>& get_generalized_velocity() const {
    return *generalized_velocity_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// generalized velocity `v`.
  StateVector<T>* get_mutable_generalized_velocity() {
    return generalized_velocity_.get();
  }

  /// Returns the subset of the state vector that is other continuous state `z`.
  const StateVector<T>& get_misc_continuous_state() const {
    return *misc_continuous_state_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// other continuous state `z`.
  StateVector<T>* get_mutable_misc_continuous_state() {
    return misc_continuous_state_.get();
  }

 private:
  /// The entire state vector.  May or may not own the underlying data.
  std::unique_ptr<StateVector<T>> state_;

  /// Generalized coordinates representing System configuration, conventionally
  /// denoted `q`. These are second-order state variables.
  /// This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<StateVector<T>> generalized_position_;

  /// Generalized speeds representing System velocity. Conventionally denoted
  /// `v`. These are first-order state variables that the System can linearly
  /// map to time derivatives `qdot` of `q` above.
  /// This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<StateVector<T>> generalized_velocity_;

  /// Additional continuous, first-order state variables not representing
  /// multibody system motion.  Conventionally denoted `z`.
  /// This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<StateVector<T>> misc_continuous_state_;
};

/// The State is a container for all the data comprising the complete state of
/// a particular System at a particular moment. Any field in the State may be
/// empty if it is not applicable to the System in question. A System may not
/// maintain state in any place other than the State object.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
struct State {
  std::unique_ptr<ContinuousState<T>> continuous_state;
  // TODO(david-german-tri): Add discrete state variables.
};

}  // namespace systems
}  // namespace drake

#pragma once

#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/state_subvector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// The ContinuousState is a container for all the State variables that are
/// unique to continuous Systems, i.e. Systems that have defined dynamics at
/// all times.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class ContinuousState {
 public:
  /// Constructs a ContinuousState for a system that does not have second-order
  /// structure: All of the state is misc_continuous_state_.
  explicit ContinuousState(std::unique_ptr<VectorBase<T>> state) {
    state_ = std::move(state);
    generalized_position_.reset(new StateSubvector<T>(state_.get()));
    generalized_velocity_.reset(new StateSubvector<T>(state_.get()));
    misc_continuous_state_.reset(
        new StateSubvector<T>(state_.get(), 0, state_->size()));
  }

  /// Constructs a ContinuousState that exposes second-order structure.
  /// The contents of @p state must be laid out as follows:
  ///
  /// <pre>
  /// (index 0)|--q--|--v--|--z--|(index state.size() - 1)
  ///
  /// Where q is generalized position
  ///       v is generalized velocity
  ///       z is other continuous state
  /// </pre>
  ///
  ///
  /// @param state The source of continuous state information.
  /// @param num_q The number of position variables.
  /// @param num_v The number of velocity variables.
  /// @param num_z The number of other variables.
  ContinuousState(std::unique_ptr<VectorBase<T>> state, int num_q, int num_v,
                  int num_z) {
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

  /// Constructs a continuous state that exposes second-order structure, with
  /// no particular constraints on the layout.
  ///
  /// @param state The entire continuous state.
  /// @param q The subset of state that is generalized position.
  /// @param v The subset of state that is generalized velocity.
  /// @param z The subset of state that is neither position nor velocity.
  ContinuousState(std::unique_ptr<VectorBase<T>> state,
                  std::unique_ptr<VectorBase<T>> q,
                  std::unique_ptr<VectorBase<T>> v,
                  std::unique_ptr<VectorBase<T>> z)
      : state_(std::move(state)),
        generalized_position_(std::move(q)),
        generalized_velocity_(std::move(v)),
        misc_continuous_state_(std::move(z)) {
    const int num_q = generalized_position_->size();
    const int num_v = generalized_velocity_->size();
    const int n = num_q + num_v + misc_continuous_state_->size();
    DRAKE_ASSERT(state_->size() == n);
    DRAKE_ASSERT(num_v <= num_q);
  }

  virtual ~ContinuousState() {}

  /// Returns the entire state vector.
  const VectorBase<T>& get_state() const { return *state_; }

  /// Returns a mutable pointer to the entire state vector, never null.
  VectorBase<T>* get_mutable_state() { return state_.get(); }

  /// Returns the subset of the state vector that is generalized position `q`.
  const VectorBase<T>& get_generalized_position() const {
    return *generalized_position_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// generalized position `q`.
  VectorBase<T>* get_mutable_generalized_position() {
    return generalized_position_.get();
  }

  /// Returns the subset of the state vector that is generalized velocity `v`.
  const VectorBase<T>& get_generalized_velocity() const {
    return *generalized_velocity_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// generalized velocity `v`.
  VectorBase<T>* get_mutable_generalized_velocity() {
    return generalized_velocity_.get();
  }

  /// Returns the subset of the state vector that is other continuous state `z`.
  const VectorBase<T>& get_misc_continuous_state() const {
    return *misc_continuous_state_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// other continuous state `z`.
  VectorBase<T>* get_mutable_misc_continuous_state() {
    return misc_continuous_state_.get();
  }

  /// Sets the entire continuous state vector from an Eigen expression.
  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == state_->size());
    this->get_mutable_state()->SetFromVector(value);
  }

  /// Returns a copy of the entire continuous state vector into an Eigen vector.
  VectorX<T> CopyToVector() const {
    return this->get_state().CopyToVector();
  }

 private:
  // The entire state vector.  May or may not own the underlying data.
  std::unique_ptr<VectorBase<T>> state_;

  // Generalized coordinates representing System configuration, conventionally
  // denoted `q`. These are second-order state variables.
  // This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<VectorBase<T>> generalized_position_;

  // Generalized speeds representing System velocity. Conventionally denoted
  // `v`. These are first-order state variables that the System can linearly
  // map to time derivatives `qdot` of `q` above.
  // This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<VectorBase<T>> generalized_velocity_;

  // Additional continuous, first-order state variables not representing
  // multibody system motion.  Conventionally denoted `z`.
  // This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<VectorBase<T>> misc_continuous_state_;
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

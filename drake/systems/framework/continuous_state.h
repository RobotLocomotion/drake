#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/subvector.h"
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
    generalized_position_.reset(new Subvector<T>(state_.get()));
    generalized_velocity_.reset(new Subvector<T>(state_.get()));
    misc_continuous_state_.reset(
        new Subvector<T>(state_.get(), 0, state_->size()));
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
    generalized_position_.reset(new Subvector<T>(state_.get(), 0, num_q));
    generalized_velocity_.reset(new Subvector<T>(state_.get(), num_q, num_v));
    misc_continuous_state_.reset(
        new Subvector<T>(state_.get(), num_q + num_v, num_z));
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

  /// Constructs a zero-length ContinuousState.
  ContinuousState()
      : ContinuousState(std::make_unique<BasicVector<T>>(0)) {}

  virtual ~ContinuousState() {}

  /// Returns the size of the entire continuous state vector.
  int size() const { return get_vector().size(); }

  T& operator[](std::size_t idx) { return (*state_)[idx]; }
  const T& operator[](std::size_t idx) const { return (*state_)[idx]; }

  /// Returns the entire continuous state vector.
  const VectorBase<T>& get_vector() const { return *state_; }

  /// Returns a mutable pointer to the entire continuous state vector, which
  /// is never nullptr.
  VectorBase<T>* get_mutable_vector() { return state_.get(); }

  /// Returns the subset of the state vector that is generalized position `q`.
  const VectorBase<T>& get_generalized_position() const {
    return *generalized_position_;
  }

  /// Returns a mutable pointer to the subset of the state vector that is
  /// generalized position `q`.
  VectorBase<T>* get_mutable_generalized_position() {
    return generalized_position_.get();
  }

  /// Returns the subset of the continuous state vector that is generalized
  /// velocity `v`.
  const VectorBase<T>& get_generalized_velocity() const {
    return *generalized_velocity_;
  }

  /// Returns a mutable pointer to the subset of the continuous state vector
  /// that is generalized velocity `v`.
  VectorBase<T>* get_mutable_generalized_velocity() {
    return generalized_velocity_.get();
  }

  /// Returns the subset of the continuous state vector that is other
  /// continuous state `z`.
  const VectorBase<T>& get_misc_continuous_state() const {
    return *misc_continuous_state_;
  }

  /// Returns a mutable pointer to the subset of the continuous state vector
  /// that is other continuous state `z`.
  VectorBase<T>* get_mutable_misc_continuous_state() {
    return misc_continuous_state_.get();
  }

  /// Sets the entire continuous state vector from an Eigen expression.
  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == state_->size());
    this->get_mutable_vector()->SetFromVector(value);
  }

  /// Returns a copy of the entire continuous state vector into an Eigen vector.
  VectorX<T> CopyToVector() const { return this->get_vector().CopyToVector(); }

  // ContinuousState is not copyable or moveable.
  ContinuousState(const ContinuousState& other) = delete;
  ContinuousState& operator=(const ContinuousState& other) = delete;
  ContinuousState(ContinuousState&& other) = delete;
  ContinuousState& operator=(ContinuousState&& other) = delete;

 private:
  // The entire continuous state vector.  May or may not own the underlying
  // data.
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

}  // namespace systems
}  // namespace drake

#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// %ContinuousState is a container for all the continuous state
/// variables `xc`. Continuous state variables are those whose values are
/// defined by differential equations, so we expect there to be a well-defined
/// time derivative `xcdot` ≜ `d/dt xc`.
///
/// The contents of `xc` are conceptually partitioned into three groups:
/// <pre>
///          |------- xc ------|
/// (index 0)|--q--|--v--|--z--|(index %xc.size() - 1)
///
/// Where q is generalized position
///       v is generalized velocity
///       z is other continuous state
/// </pre>
/// Any of the groups may be empty. However, groups q and v must be either both
/// present or both empty, because the time derivative `qdot` of the
/// second-order state variables `q` must be computable using a linear mapping
/// `qdot=N(q)*v`.
///
/// The time derivative `xcdot` has the identical substructure to `xc`, with the
/// partitions interpreted as `qdot`, `vdot`, and `zdot`. We use identical
/// %ContinuousState objects for both.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class ContinuousState {
 public:
  // ContinuousState is not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContinuousState)

  /// Constructs a ContinuousState for a system that does not have second-order
  /// structure. The `q` and `v` partitions are empty; all of the state `xc` is
  /// miscellaneous continuous state `z`.
  explicit ContinuousState(std::unique_ptr<VectorBase<T>> state) {
    state_ = std::move(state);
    generalized_position_.reset(new Subvector<T>(state_.get()));
    generalized_velocity_.reset(new Subvector<T>(state_.get()));
    misc_continuous_state_.reset(
        new Subvector<T>(state_.get(), 0, state_->size()));
    DRAKE_ASSERT_VOID(DemandInvariants());
  }

  /// Constructs a ContinuousState that exposes second-order structure.
  ///
  /// @param state The source xc of continuous state information.
  /// @param num_q The number of position variables q.
  /// @param num_v The number of velocity variables v.
  /// @param num_z The number of other continuous variables z.
  ///
  /// We require that `num_q ≥ num_v` and that the sum of the partition sizes
  /// adds up to the size of `state`.
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
    DRAKE_ASSERT_VOID(DemandInvariants());
  }

  /// Constructs a zero-length ContinuousState.
  ContinuousState()
      : ContinuousState(std::make_unique<BasicVector<T>>(0)) {}

  virtual ~ContinuousState() {}

  /// Returns the size of the entire continuous state vector.
  int size() const { return get_vector().size(); }

  T& operator[](std::size_t idx) { return (*state_)[idx]; }
  const T& operator[](std::size_t idx) const { return (*state_)[idx]; }

  /// Returns a reference to the entire continuous state vector.
  const VectorBase<T>& get_vector() const {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  /// Returns a mutable reference to the entire continuous state vector.
  VectorBase<T>& get_mutable_vector() {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_.get();
  }

  /// Returns a const reference to the subset of the state vector that is
  /// generalized position `q`. May be zero length.
  const VectorBase<T>& get_generalized_position() const {
    return *generalized_position_;
  }

  /// Returns a mutable reference to the subset of the state vector that is
  /// generalized position `q`. May be zero length.
  VectorBase<T>& get_mutable_generalized_position() {
    return *generalized_position_.get();
  }

  /// Returns a const reference to the subset of the continuous state vector
  /// that is generalized velocity `v`. May be zero length.
  const VectorBase<T>& get_generalized_velocity() const {
    return *generalized_velocity_;
  }

  /// Returns a mutable reference to the subset of the continuous state vector
  /// that is generalized velocity `v`. May be zero length.
  VectorBase<T>& get_mutable_generalized_velocity() {
    return *generalized_velocity_.get();
  }

  /// Returns a const reference to the subset of the continuous state vector
  /// that is other continuous state `z`. May be zero length.
  const VectorBase<T>& get_misc_continuous_state() const {
    return *misc_continuous_state_;
  }

  /// Returns a mutable reference to the subset of the continuous state vector
  /// that is other continuous state `z`. May be zero length.
  VectorBase<T>& get_mutable_misc_continuous_state() {
    return *misc_continuous_state_.get();
  }


  /// Copies the values from another ContinuousState of the same scalar type
  /// into this State.
  void CopyFrom(const ContinuousState<T>& other) {
    SetFromGeneric(other);
  }

  /// Initializes this ContinuousState (regardless of scalar type) from a
  /// State<double>. All scalar types in Drake must support initialization from
  /// doubles.
  void SetFrom(const ContinuousState<double>& other) {
    SetFromGeneric(other);
  }

  /// Sets the entire continuous state vector from an Eigen expression.
  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == state_->size());
    this->get_mutable_vector().SetFromVector(value);
  }

  /// Returns a copy of the entire continuous state vector into an Eigen vector.
  VectorX<T> CopyToVector() const { return this->get_vector().CopyToVector(); }

 protected:
  /// Constructs a continuous state that exposes second-order structure, with
  /// no particular constraints on the layout.
  ///
  /// @pre The q, v, z are all views into the same storage as @p state.
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
    DRAKE_ASSERT_VOID(DemandInvariants());
  }

 private:
  template <typename U>
  void SetFromGeneric(const ContinuousState<U>& other) {
    DRAKE_DEMAND(size() == other.size());
    DRAKE_DEMAND(get_generalized_position().size() ==
        other.get_generalized_position().size());
    DRAKE_DEMAND(get_generalized_velocity().size() ==
        other.get_generalized_velocity().size());
    DRAKE_DEMAND(get_misc_continuous_state().size() ==
        other.get_misc_continuous_state().size());
    SetFromVector(other.CopyToVector().template cast<T>());
  }

  // Demand that the representation invariants hold.
  void DemandInvariants() const {
    // Nothing is nullptr.
    DRAKE_DEMAND(!!generalized_position_);
    DRAKE_DEMAND(!!generalized_velocity_);
    DRAKE_DEMAND(!!misc_continuous_state_);

    // The sizes are consistent.
    const int num_q = generalized_position_->size();
    const int num_v = generalized_velocity_->size();
    const int num_z = misc_continuous_state_->size();
    const int num_total = (num_q + num_v + num_z);
    DRAKE_DEMAND(num_q >= 0);
    DRAKE_DEMAND(num_v >= 0);
    DRAKE_DEMAND(num_z >= 0);
    DRAKE_DEMAND(num_v <= num_q);
    DRAKE_DEMAND(state_->size() == num_total);

    // The storage addresses of `state_` elements contain no duplicates.
    std::unordered_set<const T*> state_element_pointers;
    for (int i = 0; i < num_total; ++i) {
      const T* element = &(state_->GetAtIndex(i));
      state_element_pointers.emplace(element);
    }
    DRAKE_DEMAND(static_cast<int>(state_element_pointers.size()) == num_total);

    // The storage addresses of (q, v, z) elements contain no duplicates, and
    // are drawn from the set of storage addresses of `state_` elements.
    // Therefore, the `state_` vector and (q, v, z) vectors form views into the
    // same unique underlying data, just with different indexing.
    std::unordered_set<const T*> qvz_element_pointers;
    for (int i = 0; i < num_q; ++i) {
      const T* element = &(generalized_position_->GetAtIndex(i));
      qvz_element_pointers.emplace(element);
      DRAKE_DEMAND(state_element_pointers.count(element) == 1);
    }
    for (int i = 0; i < num_v; ++i) {
      const T* element = &(generalized_velocity_->GetAtIndex(i));
      qvz_element_pointers.emplace(element);
      DRAKE_DEMAND(state_element_pointers.count(element) == 1);
    }
    for (int i = 0; i < num_z; ++i) {
      const T* element = &(misc_continuous_state_->GetAtIndex(i));
      qvz_element_pointers.emplace(element);
      DRAKE_DEMAND(state_element_pointers.count(element) == 1);
    }
    DRAKE_DEMAND(static_cast<int>(qvz_element_pointers.size()) == num_total);
  }

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

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The states in the FEM simulation that are associated with the nodes and the
 elements. The states include the generalized positions associated with each
 node, `q`, and optionally, their first and second time derivatives, `qdot` and
 `qddot`. %FemState also contains the cache entries that are associated with the
 elements whose values depend on the states. See ElementCacheEntry for more on
 these state-dependent quantities.
 @tparam_nonsymbolic_scalar T. */
template <typename Element>
class FemState {
 public:
  using T = typename Element::T;
  using ElementCacheEntryType = typename Element::ElementCacheEntryType;

  /** Copy, move and assign are disabled. Copy constructors and assignment are
   usually expensive for %FemState. If a copy is required, use Clone()
   if a deep copy is required. Use SetFrom() if you want to set `this`
   %FemState from another %FemState. */
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemState);

  /** The order of the ODE problem after FEM spatial discretization. */
  static constexpr int ode_order() { return Element::ode_order(); }

  /** Constructs an %FemState of a zero-th order equation with prescribed
   generalized positions.
   @param[in] q The prescribed generalized positions.
   @pre ode_order() == 0. */
  explicit FemState(const Eigen::Ref<const VectorX<T>>& q) : q_(q) {
    DRAKE_DEMAND(ode_order() == 0);
    DRAKE_DEMAND(qdot_.size() == 0);
    DRAKE_DEMAND(qddot_.size() == 0);
  }

  /** Constructs an %FemState of a first order equation with prescribed
   generalized positions and their time derivatives.
   @param[in] q The prescribed generalized positions.
   @param[in] qdot The prescribed time derivatives of generalized positions.
   @pre ode_order() == 1.
   @pre `q` and `qdot` must have the same size. */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot)
      : q_(q), qdot_(qdot) {
    DRAKE_DEMAND(ode_order() == 1);
    DRAKE_DEMAND(q.size() == qdot.size());
    DRAKE_DEMAND(qddot_.size() == 0);
  }

  /** Constructs an %FemState of a second order equation with prescribed
   generalized positions and their first and second order time derivatives.
   @param[in] q The prescribed generalized positions.
   @param[in] qdot The prescribed time derivatives of generalized positions.
   @param[in] qddot The prescribed time second derivatives of generalized
   positions.
   @pre ode_order() == 2.
   @pre `q`, `qdot` and `qddot` must have the same size. */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot,
           const Eigen::Ref<const VectorX<T>>& qddot)
      : q_(q), qdot_(qdot) {
    DRAKE_DEMAND(ode_order() == 2);
    DRAKE_DEMAND(q.size() == qdot.size());
    DRAKE_DEMAND(q.size() == qddot.size());
  }

  /** Takes ownership of the input `element_cache` and set it as the element
   cache that `this` %FemState owns. This method is usually called
   right after the construction of the %FemState to set the element
   cache to the desired quantity. */
  void ResetElementCache(
      const std::vector<ElementCacheEntryType>& element_cache) {
    element_cache_ = element_cache;
  }

  /** Creates a deep identical copy of `this` %FemState with all its
   states and cache. Returns a unique pointer to the copy. */
  std::unique_ptr<FemState<Element>> Clone() const {
    std::unique_ptr<FemState<Element>> clone;
    if constexpr (ode_order() == 0)
      clone = std::make_unique<FemState<Element>>(q());
    else if constexpr (ode_order() == 1)
      clone = std::make_unique<FemState<Element>>(q(), qdot());
    else if constexpr (ode_order() == 2)
      clone = std::make_unique<FemState<Element>>(q(), qdot(), qddot());
    clone->element_cache_ = element_cache_;
    return clone;
  }

  /** Sets the states and cache of `this` %FemState from the input
   %FemState `other`. After this method is called, the state and cache
   in `this` %FemState will be an identical copy of those in the
   `other` %FemState. */
  void SetFrom(const FemState<Element>& other) {
    Resize(other.num_generalized_positions());
    set_q(other.q());
    if constexpr (ode_order() >= 1) set_qdot(other.qdot());
    if constexpr (ode_order() == 2) set_qddot(other.qddot());
    element_cache_ = other.element_cache_;
  }

  /** Resize the number of states to the input `num_generalized_positions`. The
   existing values are unchanged if `num_generalized_positions` is greater than
   or equal to the number of existing generalized positions. */
  void Resize(int num_generalized_positions) {
    DRAKE_ASSERT(num_generalized_positions >= 0);
    q_.conservativeResize(num_generalized_positions);
    if constexpr (ode_order() >= 1)
      qdot_.conservativeResize(num_generalized_positions);
    if constexpr (ode_order() == 2)
      qddot_.conservativeResize(num_generalized_positions);
  }

  /** @name State getters.
   @{ */
  const VectorX<T>& q() const { return q_; }

  const VectorX<T>& qdot() const {
    DRAKE_DEMAND(ode_order() >= 1);
    return qdot_;
  }

  const VectorX<T>& qddot() const {
    DRAKE_DEMAND(ode_order() == 2);
    return qddot_;
  }
  /** @} */

  /** @name State setters.
   The size of the values provided must match the current size
   of the states.
   @{ */
  void set_q(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == q_.size());
    mutable_q() = value;
  }

  void set_qdot(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(ode_order() >= 1);
    DRAKE_ASSERT(value.size() == qdot_.size());
    mutable_qdot() = value;
  }

  void set_qddot(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(ode_order() == 2);
    DRAKE_ASSERT(value.size() == qddot_.size());
    mutable_qddot() = value;
  }
  /** @} */

  /** @name Mutable state getters.
   The values of the states are mutable but the sizes
   of the states are not allowed to change.
   @{ */
  Eigen::VectorBlock<VectorX<T>> mutable_q() { return q_.head(q_.size()); }

  Eigen::VectorBlock<VectorX<T>> mutable_qdot() {
    DRAKE_DEMAND(ode_order() >= 1);
    return qdot_.head(qdot_.size());
  }

  Eigen::VectorBlock<VectorX<T>> mutable_qddot() {
    DRAKE_DEMAND(ode_order() == 2);
    return qddot_.head(qddot_.size());
  }
  /** @} */

  /** @name Getters and mutable getters for cache entries.
   @{ */
  const ElementCacheEntryType& element_cache_entry(ElementIndex e) const {
    DRAKE_ASSERT(e.is_valid());
    DRAKE_ASSERT(e < element_cache_.size());
    return element_cache_[e];
  }

  ElementCacheEntryType& mutable_element_cache_entry(ElementIndex e) const {
    DRAKE_ASSERT(e.is_valid());
    DRAKE_ASSERT(e < element_cache_.size());
    return element_cache_[e];
  }
  /** @} */

  int num_generalized_positions() const { return q_.size(); }

  int element_cache_size() const { return element_cache_.size(); }

 private:
  // Generalized node positions.
  VectorX<T> q_{};
  // Time derivatives of generalized node positions.
  VectorX<T> qdot_{};
  // Time second derivatives of generalized node positions.
  VectorX<T> qddot_{};
  // Owned element cache entries.
  mutable std::vector<ElementCacheEntryType> element_cache_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake

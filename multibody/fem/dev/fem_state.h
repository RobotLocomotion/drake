#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/element_cache_entry.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** The states in the FEM simulation that are associated with the nodes and the
 elements. The states include the generalized positions associated with each
 node, `q`, and their time derivatives, `qdot`. %FemState also contains the
 cache entries that are associated with the elements whose values depend on
 the states. See ElementCacheEntry for more on these state-dependent quantities.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class FemState {
 public:
  /** Copy, move and assign are disabled. Copy constructors and assignment are
   usually expensive for %FemState. If a copy is required, use Clone() if a deep
   copy is required. Use SetFrom() if you want to set `this` %FemState from
   another %FemState. */
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemState);

  /** Constructs an %FemState with prescribed states.
   @param[in] q The prescribed generalized positions.
   @param[in] qdot The prescribed time derivatives of generalized positions.
   @pre `q` and `qdot` must have the same size. */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot)
      : q_(q), qdot_(qdot) {
    DRAKE_DEMAND(q.size() == qdot.size());
  }

  /** Takes ownership of the input `element_cache` and set it as the element
   cache that `this` %FemState owns. This method is usually called right after
   the construction of the %FemState to set the element cache to the desired
   quantity. */
  void ResetElementCache(
      std::vector<std::unique_ptr<ElementCacheEntry<T>>> element_cache) {
    element_cache_.resize(element_cache.size());
    for (int i = 0; i < static_cast<int>(element_cache.size()); ++i) {
      element_cache_[i] = std::move(element_cache[i]);
    }
  }

  /** Creates a deep identical copy of `this` %FemState with all its states and
   cache. Returns a unique pointer to the copy. */
  std::unique_ptr<FemState<T>> Clone() const {
    auto clone = std::make_unique<FemState<T>>(q(), qdot());
    clone->element_cache_ = element_cache_;
    return clone;
  }

  /** Sets the states and cache of `this` %FemState from the input %FemState
   `other`. After this method is called, the state and cache in `this`
   %FemState will be an identical copy of those in the `other` %FemState. */
  void SetFrom(const FemState<T>& other) {
    Resize(other.num_generalized_positions());
    set_q(other.q());
    set_qdot(other.qdot());
    element_cache_ = other.element_cache_;
  }

  /** Resize the number of states to the input `num_generalized_positions`. The
   existing values are unchanged if `num_generalized_positions` is greater than
   or equal to the number of existing generalized positions. */
  void Resize(int num_generalized_positions) {
    DRAKE_ASSERT(num_generalized_positions >= 0);
    if (num_generalized_positions == this->num_generalized_positions()) return;
    qdot_.conservativeResize(num_generalized_positions);
    q_.conservativeResize(num_generalized_positions);
  }

  /** @name State getters.
   @{ */
  const VectorX<T>& qdot() const { return qdot_; }

  const VectorX<T>& q() const { return q_; }
  /** @} */

  /** @name State setters.
   The size of the values provided must match the current size
   of the states.
   @{ */
  void set_qdot(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == qdot_.size());
    mutable_qdot() = value;
  }

  void set_q(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == q_.size());
    mutable_q() = value;
  }
  /** @} */

  /** @name Mutable state getters.
   The values of the states are mutable but the sizes
   of the states are not allowed to change.
   @{ */
  Eigen::VectorBlock<VectorX<T>> mutable_qdot() {
    return qdot_.head(qdot_.size());
  }

  Eigen::VectorBlock<VectorX<T>> mutable_q() { return q_.head(q_.size()); }
  /** @} */

  /** @name Getters and mutable getters for cache entries.
   @{ */
  const ElementCacheEntry<T>& element_cache_entry(ElementIndex e) const {
    DRAKE_ASSERT(e.is_valid());
    DRAKE_ASSERT(e < element_cache_.size());
    DRAKE_ASSERT(element_cache_[e] != nullptr);
    return *element_cache_[e];
  }

  ElementCacheEntry<T>& mutable_element_cache_entry(ElementIndex e) const {
    DRAKE_ASSERT(e.is_valid());
    DRAKE_ASSERT(e < element_cache_.size());
    DRAKE_ASSERT(element_cache_[e] != nullptr);
    return *element_cache_[e];
  }
  /** @} */

  int num_generalized_positions() const { return q_.size(); }

  int element_cache_size() const { return element_cache_.size(); }

 private:
  // Generalized node positions.
  VectorX<T> q_;
  // Time derivatives of generalized node positions.
  VectorX<T> qdot_;
  // Owned element cache entries.
  std::vector<copyable_unique_ptr<ElementCacheEntry<T>>> element_cache_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake

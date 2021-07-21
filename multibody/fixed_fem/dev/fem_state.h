#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_state_base.h"

namespace drake {
namespace multibody {
namespace fem {

/** %FemState implements FemStateBase for a particular type of FemElement.
 It is templated on the concrete FemElement type in order to allow compile time
 optimizations based on fixed sizes. It also stores the per-element
 state-dependent quantities for its corresponding elements (see
 ElementCacheEntry).
 @tparam Element The type of FemElement that consumes this %FemState. This
 template parameter provides the scalar type, the type of per-element data
 this %FemState stores and the order of the ODE after FEM spatial
 discretization. */
template <typename Element>
class FemState : public FemStateBase<typename Element::T> {
 public:
  using T = typename Element::T;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  /** The order of the ODE problem after FEM spatial discretization. */
  static constexpr int ode_order() {
    constexpr int order = Element::Traits::kOdeOrder;
    static_assert(order == 0 || order == 1 || order == 2);
    return order;
  }

  /** Constructs an %FemState of a zero-th order equation with prescribed
   generalized positions.
   @param[in] q    The prescribed generalized positions.
   @pre ode_order() == 0. */
  explicit FemState(const Eigen::Ref<const VectorX<T>>& q)
      : FemStateBase<T>(q) {
    DRAKE_THROW_UNLESS(ode_order() == 0);
  }

  /** Constructs an %FemState of a first order equation with prescribed
   generalized positions and their time derivatives.
   @param[in] q    The prescribed generalized positions.
   @param[in] qdot    The prescribed time derivatives of generalized positions.
   @pre ode_order() == 1.
   @pre q.size() == qdot.size(). */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot)
      : FemStateBase<T>(q, qdot) {
    DRAKE_THROW_UNLESS(ode_order() == 1);
  }

  /** Constructs an %FemState of a second order equation with prescribed
   generalized positions and their first and second order time derivatives.
   @param[in] q    The prescribed generalized positions.
   @param[in] qdot    The prescribed time derivatives of generalized positions.
   @param[in] qddot    The prescribed time second derivatives of generalized
   positions.
   @pre ode_order() == 2.
   @pre q.size() == qdot.size().
   @pre q.size() == qddot.size(). */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot,
           const Eigen::Ref<const VectorX<T>>& qddot)
      : FemStateBase<T>(q, qdot, qddot) {
    DRAKE_THROW_UNLESS(ode_order() == 2);
  }

  /** Creates the per-element state-dependent data for the given `elements`. The
  following invariant must be satisfied: `elements[i].element_index() == i` --
  the i-th element reports an element index of `i`, as %FemState assumes this
  invariant when the Element::Traits::Data are accessed via element_data().
  @throw std::exception if elements[i].element_index() != i for some `i` = 0,
  ..., `element.size()-1`. */
  void MakeElementData(const std::vector<Element>& elements) {
    /* Note: the element data is stored in a simple bespoke cache (see
     internal::ElementCacheEntry). The newly created cache entries are
     initially stale. */
    element_cache_.clear();
    for (int i = 0; i < static_cast<int>(elements.size()); ++i) {
      if (elements[i].element_index() != ElementIndex(i)) {
        throw std::runtime_error(
            "Input element entry at " + std::to_string(i) + " has index " +
            std::to_string(elements[i].element_index()) + " instead of " +
            std::to_string(i) +
            ". The entry with index i must be stored at position i.");
      }
    }
    element_cache_.resize(elements.size());
  }

  int element_cache_size() const { return element_cache_.size(); }

  /** Getter for element state-dependpent quantities. */
  const typename Element::Traits::Data& element_data(
      const Element& element) const {
    ElementIndex id = element.element_index();
    DRAKE_ASSERT(id.is_valid() && id < element_cache_size());
    typename Element::Traits::Data& data =
        element_cache_[id].mutable_element_data();
    if (element_cache_[id].is_stale()) {
      data = element.ComputeData(*this);
      element_cache_[id].set_stale(false);
    }
    return data;
  }

 private:
  friend class FemStateTest;

  std::unique_ptr<FemStateBase<T>> DoClone() const final {
    return std::make_unique<FemState<Element>>(*this);
  }

  // TODO(xuchenhan-tri): Currently, all cache entries are thrashed when *any*
  //  state (q, qdot, or qddot) is changed. For many FEM models (e.g.
  //  static/dynamic elasticity), there exist more fine-grained caching
  //  mechanisms which may improve the cache efficiency.
  /* Mark all cache entries associated with `this` FemState as stale. */
  void InvalidateAllCacheEntries() final {
    for (auto& element_cache_entry : element_cache_) {
      element_cache_entry.set_stale(true);
    }
  }

  /* Owned element cache entries. */
  mutable std::vector<
      internal::ElementCacheEntry<typename Element::Traits::Data>>
      element_cache_{};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake

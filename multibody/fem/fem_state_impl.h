#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/element_cache_entry.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* FemStateImpl implements FemState for a particular type of FEM element.
 It is templated on the concrete FemElement type in order to allow compile time
 optimizations based on fixed sizes. It also stores the per-element
 state-dependent quantities for its corresponding elements (see
 ElementCacheEntry).
 @tparam Element The type of FemElement that consumes this FemStateImpl. This
 template parameter provides the scalar type and the type of per-element data
 this FemStateImpl stores. */
template <typename Element>
class FemStateImpl : public FemState<typename Element::T> {
 public:
  using T = typename Element::T;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemStateImpl);

  /* Constructs an FemStateImpl with the prescribed positions, velocities, and
   accelerations.
   @param[in] q  The prescribed generalized positions.
   @param[in] v  The prescribed generalized velocities.
   @param[in] a  The prescribed generalized accelerations.
   positions.
   @pre q.size() == v.size().
   @pre q.size() == a.size(). */
  FemStateImpl(const Eigen::Ref<const VectorX<T>>& q,
               const Eigen::Ref<const VectorX<T>>& v,
               const Eigen::Ref<const VectorX<T>>& a)
      : FemState<T>(q, v, a) {}

  /* Creates the per-element state-dependent data for the given `elements`. The
  following invariant must be satisfied: `elements[i].element_index() == i` --
  the i-th element reports an element index of `i`, as FemStateImpl assumes this
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

  /* Returns the number of element cache entries. */
  int element_cache_size() const { return element_cache_.size(); }

  /* Getter for element state-dependpent quantities. */
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

  // TODO(xuchenhan-tri): Currently, all cache entries are thrashed when *any*
  //  state (q, v, or a) is changed. For most FEM models, there exist more
  //  fine-grained caching mechanisms which may improve the cache efficiency.
  /* Mark all cache entries associated with `this` FemStateImpl as stale. */
  void InvalidateAllCacheEntries() final {
    for (auto& element_cache_entry : element_cache_) {
      element_cache_entry.set_stale(true);
    }
  }

  /* Owned element cache entries. */
  mutable std::vector<ElementCacheEntry<typename Element::Traits::Data>>
      element_cache_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

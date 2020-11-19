#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** %ElementCacheEntry stores the per element state-dependent quantities used in
 an FEM simulation that are not states themselves. %ElementCacheEntry should be
 used in tandem with FemElement. There should be a one-to-one correspondence
 between each FemElement that performs the element routine and each
 %ElementCacheEntry that stores the state-dependent quantities used in the
 routine. This correspondence is maintained by the shared element index that is
 assigned to both the FemElement and the %ElementCacheEntry in correspondence.
 Furthermore, the type of FemElement and %ElementCacheEntry in correspondence
 must be compatible. More specifically, if the FemElement is of concrete type
 `FooElement`, then the %ElementCacheEntry that shares the same element index
 must be of concrete type `FooElementCacheEntry`.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElementCacheEntry {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  ElementCacheEntry(ElementCacheEntry&&) = delete;
  ElementCacheEntry& operator=(ElementCacheEntry&&) = delete;
  ElementCacheEntry& operator=(const ElementCacheEntry&) = delete;
  /** @} */

  virtual ~ElementCacheEntry() = default;

  /** Creates an identical copy of the concrete ElementCacheEntry object.
   */
  std::unique_ptr<ElementCacheEntry<T>> Clone() const { return DoClone(); }

  /** The index of the FemElement associated with this %ElementCacheEntry. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which the cache entry needs to be
   evaluated. */
  int num_quadrature_points() const { return num_quadrature_points_; }

  // TODO(xuchenhan-tri): Add interface for marking cache entries stale when
  // caching is in place.

 protected:
  /** Constructs a new ElementCacheEntry.
   @param element_index The index of the FemElement associated with this
   ElementCacheEntry.
   @param num_quadrature_points The number of quadrature locations at which
   the cache entry needs to be evaluated.
   @pre `num_quadrature_points` must be positive. */
  ElementCacheEntry(ElementIndex element_index, int num_quadrature_points)
      : element_index_(element_index),
        num_quadrature_points_(num_quadrature_points) {
    DRAKE_ASSERT(element_index_.is_valid());
    DRAKE_ASSERT(num_quadrature_points > 0);
  }

  /** Copy constructor for the base ElementCacheEntry class to facilitate
   `DoClone()` in derived classes. */
  ElementCacheEntry(const ElementCacheEntry&) = default;

  /** Creates an identical copy of the concrete ElementCacheEntry object.
   Derived classes must implement this so that it performs the complete
   deep copy of the object, including all base class members. */
  virtual std::unique_ptr<ElementCacheEntry<T>> DoClone() const = 0;

 private:
  ElementIndex element_index_;
  int num_quadrature_points_{-1};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake

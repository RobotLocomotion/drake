#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** %ElementCache stores the per element state-dependent quantities used in an
 FEM simulation that are not states themselves. %ElementCache should be used in
 tandem with FemElement. There should be a one-to-one correspondence between
 each FemElement that performs the element routine and each %ElementCache that
 stores the state-dependent quantities used in the routine. This correspondence
 is maintained by the shared element index that is assigned to both the
 FemElement and the %ElementCache in correspondence. Furthermore, the type of
 FemElement and %ElementCache in correspondence must be compatible. More
 specifically, if the FemElement is of concrete type `FooElement`, then the
 %ElementCache that shares the same element index must be of concrete type
 `FooElementCache`.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElementCache {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  ElementCache(ElementCache&&) = delete;
  ElementCache& operator=(ElementCache&&) = delete;
  ElementCache& operator=(const ElementCache&) = delete;
  /** @} */

  virtual ~ElementCache() = default;

  /** Creates an identical copy of the concrete ElementCache object.
   */
  std::unique_ptr<ElementCache<T>> Clone() const { return DoClone(); }

  /** The index of the FemElement associated with this %ElementCache. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which cached quantities need to be
   evaluated. */
  int num_quadrature_points() const { return num_quadrature_points_; }

  // TODO(xuchenhan-tri): Add interface for marking cache entries stale when
  // caching is in place.

 protected:
  /** Constructs a new ElementCache.
   @param element_index The index of the FemElement associated with this
   ElementCache.
   @param num_quadrature_points The number of quadrature locations at which
   cached quantities need to be evaluated.
   @pre `num_quadrature_points` must be positive. */
  ElementCache(ElementIndex element_index, int num_quadrature_points)
      : element_index_(element_index),
        num_quadrature_points_(num_quadrature_points) {
    DRAKE_ASSERT(element_index_.is_valid());
    DRAKE_ASSERT(num_quadrature_points > 0);
  }

  /** Copy constructor for the base ElementCache class to facilitate
   `DoClone()` in derived classes. */
  ElementCache(const ElementCache&) = default;

  /** Creates an identical copy of the concrete ElementCache object.
   Derived classes must implement this so that it performs the complete
   deep copy of the object, including all base class members. */
  virtual std::unique_ptr<ElementCache<T>> DoClone() const = 0;

 private:
  ElementIndex element_index_;
  int num_quadrature_points_{-1};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake

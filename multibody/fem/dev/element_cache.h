#pragma once

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
 is maintained by the same element index that is assigned to the FemElement
 and the %ElementCache in correspondence. Furthermore, the type of FemElement
 and %ElementCache in correspondence must be compatible. More specifically, if
 the FemElement is of concrete type `FemFoo`, then the %ElementCache that shares
 the same element index must be of concrete type `FooElementCache`.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElementCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElementCache);

  virtual ~ElementCache() = default;

  /** The index of the FemElement associated with this %ElementCache. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which cached quantities need to be
   evaluated. */
  int num_quads() const { return num_quads_; }

  // TODO(xuchenhan-tri): Add interface for marking cache entries stale when
  // caching is in place.

 protected:
  /* Constructs a new ElementCache.
   @param element_index The index of the FemElement associated with this
   ElementCache.
   @param num_quads The number of quadrature locations at which cached
   quantities need to be evaluated.
   @pre `num_quads` must be positive. */
  ElementCache(ElementIndex element_index, int num_quads)
      : element_index_(element_index), num_quads_(num_quads) {
    DRAKE_DEMAND(element_index_.is_valid());
    DRAKE_DEMAND(num_quads > 0);
  }

 private:
  ElementIndex element_index_;
  int num_quads_{-1};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake

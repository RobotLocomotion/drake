#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %ElementCacheEntry stores the per element state-dependent quantities used in
 an FEM simulation that are not states themselves. %ElementCacheEntry should be
 used in tandem with FemElement. There should be a one-to-one correspondence
 between each FemElement that performs the element routine and each
 %ElementCacheEntry that stores the state-dependent quantities used in the
 routine. This correspondence is maintained by the shared element index that is
 assigned to both the FemElement and the %ElementCacheEntry in correspondence.
 Furthermore, the type of FemElement and %ElementCacheEntry in correspondence
 must be compatible. More specifically, if the FemElement is of type
 `FooElement`, then the %ElementCacheEntry that shares the same element index
 must be of concrete type `FooElementCacheEntry`. Each concrete FemElement (e.g.
 StaticElasticityElement) declares its compatible %ElementCacheEntry through a
 alias declaration `ElementCacheEntryType`. */
class ElementCacheEntry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElementCacheEntry);

  ~ElementCacheEntry() = default;

  /** The index of the FemElement associated with this %ElementCacheEntry. */
  ElementIndex element_index() const { return element_index_; }

  // TODO(xuchenhan-tri): Add interface for marking cache entries stale when
  //  caching is in place.

 protected:
  /** Constructs a new ElementCacheEntry.
   @param element_index The index of the FemElement associated with this
   ElementCacheEntry. */
  explicit ElementCacheEntry(ElementIndex element_index)
      : element_index_(element_index) {
    DRAKE_ASSERT(element_index_.is_valid());
  }

 private:
  ElementIndex element_index_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake

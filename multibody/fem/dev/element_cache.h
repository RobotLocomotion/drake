#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
/** This class stores the per element cached quantities that depend on FemState
 to save computation time when we know that these quantities have not changed
 since the last time we computed them. When the `Foo` state in FemState
 changes, the method `mark_Foo_cache_stale` needs to be called to to invalid
 the cache entries that depend on `Foo`.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
template <typename T, int SpatialDim>
class ElementCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElementCache);

  ElementCache(int element_index, int num_quads)
      : element_index_(element_index), num_quads_(num_quads) {}

  virtual ~ElementCache() = default;

  /// Marks all cached quantities that depend on v0 as stale.
  virtual void mark_v0_cache_stale() = 0;

  /// Marks all cached quantities that depend on v as stale.
  virtual void mark_v_cache_stale() = 0;

  /// Marks all cached quantities that depend on x0 as stale.
  virtual void mark_x0_cache_stale() = 0;

  /// Marks all cached quantities that depend on x as stale.
  virtual void mark_x_cache_stale() = 0;

  int element_index() const { return element_index_; }
  int num_quads() const { return num_quads_; }

 private:
  // The global index of the element whose quantities `this` element cache
  // caches.
  int element_index_{-1};
  // Number of quadrature points used on this element.
  int num_quads_{-1};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake

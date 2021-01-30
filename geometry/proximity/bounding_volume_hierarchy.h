#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/proximity/bvh.h"

namespace drake {
namespace geometry {
namespace internal {

template <class MeshType>
using BoundingVolumeHierarchy DRAKE_DEPRECATED(
    "2021-01-01",
    "Use Bvh instead of BoundingVolumeHierarchy.") = Bvh<MeshType>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake

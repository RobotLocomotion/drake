#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/** Type used to locate a dynamic geometry in the geometry engine. */
using GeometryIndex = TypeSafeIndex<class GeometryTag>;

/** Type used to locate an anchored geometry in the geometry engine. */
using AnchoredGeometryIndex = TypeSafeIndex<class AnchoredGeometryTag>;

/** Type used to locate a frame pose in the geometry state. */
using PoseIndex = TypeSafeIndex<class GeometryPoseTag>;

}  // namespace geometry
}  // namespace drake

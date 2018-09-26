#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {


/** Index used to identify a geometry in the proximity engine. The same index
 type applies to both anchored and dynamic geometries. They are distinguished
 by which method they are passed to.  */
using ProximityIndex = TypeSafeIndex<class ProximityTag>;

/** Index into the global set of internal entities -- spanning all roles. The
 index can refer to either a geometry, or a frame -- the context determines
 which is the correct interpretation.  In other words, the first
 InternalGeometry and the first InternalFrame will *both* have the same
 InternalIndex value.  */
using InternalIndex = TypeSafeIndex<class InternalGeometryTag>;

}  // namespace geometry
}  // namespace drake

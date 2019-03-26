#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/** Type used to locate any geometry in the render engine. */
using RenderIndex = TypeSafeIndex<class RenderTag>;

/** Index used to identify a geometry in the proximity engine. The same index
 type applies to both anchored and dynamic geometries. They are distinguished
 by which method they are passed to.  */
using ProximityIndex = TypeSafeIndex<class ProximityTag>;

/** Index into the ordered vector of all registered geometries (regardless of
 parent frame, role, etc.)   */
using GeometryIndex = TypeSafeIndex<class GeometryTag>;

/** Index into the ordered vector of all registered frames -- by convention,
 the world frame's index is always zero.  */
using FrameIndex = TypeSafeIndex<class GeometryTag>;

}  // namespace geometry
}  // namespace drake

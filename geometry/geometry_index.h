#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): Remove FrameIndex.
/** Index into the ordered vector of all registered frames -- by convention,
 the world frame's index is always zero.  */
using FrameIndex = TypeSafeIndex<class GeometryTag>;

}  // namespace geometry
}  // namespace drake

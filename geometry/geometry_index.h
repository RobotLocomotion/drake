#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

// TODO(2022-01-01): Remove this header when removing the deprecated type.
/** Index into the ordered vector of all registered frames -- by convention,
 the world frame's index is always zero.  */
using FrameIndex DRAKE_DEPRECATED("2022-01-01", "This is an unused type.") =
    TypeSafeIndex<class GeometryTag>;

}  // namespace geometry
}  // namespace drake

#pragma once

#include "drake/geometry/identifier.h"

namespace drake {
namespace geometry {

/** Type used to identify geometry sources in GeometrySystem. */
using SourceId = Identifier<class SourceTag>;

/** Type used to identify geometry frames in GeometrySystem .*/
using FrameId = Identifier<class FrameTag>;

/** Type used to identify geometry instances in GeometrySystem. */
using GeometryId = Identifier<class GeometryTag>;

}  // namespace geometry
}  // namespace drake

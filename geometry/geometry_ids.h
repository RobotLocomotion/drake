#pragma once

#include "drake/geometry/identifier.h"

namespace drake {
namespace geometry {

/** Type used to identify geometry sources in SceneGraph. */
using SourceId = Identifier<class SourceTag>;

/** Type used to identify geometry frames in SceneGraph.*/
using FrameId = Identifier<class FrameTag>;

/** Type used to identify geometry instances in SceneGraph. */
using GeometryId = Identifier<class GeometryTag>;

}  // namespace geometry
}  // namespace drake

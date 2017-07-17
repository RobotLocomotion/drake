#pragma once

#include "drake/geometry/identifier.h"

namespace drake {
namespace geometry {

/// Type used to identify geometry sources by index in GeometryWorld.
using SourceId = Identifier<class SourceTag>;

/// Type used to identify a frame kinematics instance by index in
/// GeometryWorld
using FrameId = Identifier<class FrameTag>;

/// Type used to identify a geometry instance by index in GeometryWorld.
using GeometryId = Identifier<class GeometryTag>;

}  // namespace geometry
}  // namespace drake

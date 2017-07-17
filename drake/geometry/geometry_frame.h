#pragma once

#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** This simple class carries the definition of a frame used by GeometryWorld.
 To register moving frames with GeometryWorld (see
 GeometryWorld::RegisterFrame), a geometry source (see
 GeometryWorld::RegisterNewSource) instantiates a frame and passes ownership
 over to GeometryWorld.

 A frame is defined by three pieces of information:
    - the name, which must be unique within a single geometry source,
    - the "frame group", an integer identifier that can be used to group frames
      together within a geometry source, and
    - the initial pose of the frame (measured and expressed in its parent
      frame). The parent is defined at registration. This is only the _initial_
      pose; registered frames are expected to move with time.

 @internal The "frame group" is intended as a generic synonym for the model
 instance id defined by the RigidBodyTree and used again in automotive to
 serve as unique car identifiers.

 @see GeometryWorld

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryFrame {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryFrame)

  GeometryFrame() = default;
};
}  // namespace geometry
}  // namespace drake

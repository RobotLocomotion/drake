#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** This simple class carries the definition of a frame used in the
 SceneGraph. To register moving frames with SceneGraph (see
 SceneGraph::RegisterFrame()), a geometry source (see
 SceneGraph::RegisterSource()) instantiates a frame and passes ownership
 over to SceneGraph.

 A frame is defined by two pieces of information:

    - the name, which must be unique within a single geometry source and
    - the "frame group", an integer identifier that can be used to group frames
      together within a geometry source.

 @internal The "frame group" is intended as a generic synonym for the model
 instance id defined by the RigidBodyTree.

 @see SceneGraph */
class GeometryFrame {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryFrame)

  /** Constructor.
   @param frame_name        The name of the frame.
   @param frame_group_id    The optional frame group identifier. If unspecified,
                            defaults to the common, 0 group. Must be
                            non-negative.  */
  explicit GeometryFrame(const std::string& frame_name, int frame_group_id = 0);

  /** Returns the globally unique id for this geometry specification. Every
   instantiation of %FrameInstance will contain a unique id value. The id
   value is preserved across copies. After successfully registering this
   %FrameInstance, this id will serve as the identifier for the registered
   representation as well. */
  FrameId id() const { return id_; }

  const std::string& name() const { return name_; }

  int frame_group() const { return frame_group_; }

 private:
  // The *globally* unique identifier for this instance. It is functionally
  // const (i.e. defined in construction) but not marked const to allow for
  // default copying/assigning.
  FrameId id_{};

  // The name of the frame. Must be unique across frames from the same geometry
  // source.
  std::string name_;

  // TODO(SeanCurtis-TRI): Consider whether this should be an Identifier or
  // TypeSafeIndex type.
  // The frame group to which this frame belongs.
  int frame_group_;
};

}  // namespace geometry
}  // namespace drake

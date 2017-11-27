#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

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

 @see GeometryWorld */
class GeometryFrame {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryFrame)

  /** Constructor.
   @param frame_name        The name of the frame.
   @param X_PF              The initial pose of this frame F, measured and
                            expressed in the _intended_ parent frame P.
   @param frame_group_id    The optional frame group identifier. If unspecified,
                            defaults to the common, 0 group. */
  GeometryFrame(const std::string& frame_name, const Isometry3<double>& X_PF,
                int frame_group_id = 0)
      : id_(FrameId::get_new_id()),
        name_(frame_name),
        X_PF_(X_PF),
        frame_group_(frame_group_id) {}

  /** Returns the globally unique id for this geometry specification. Every
   instantiation of %FrameInstance will contain a unique id value. The id
   value is preserved across copies. After successfully registering this
   %FrameInstance, this id will serve as the identifier for the registered
   representation as well. */
  FrameId id() const { return id_; }

  const std::string& name() const { return name_; }

  const Isometry3<double>& pose() const { return X_PF_; }

  int frame_group() const { return frame_group_; }

 private:
  // The *globally* unique identifier for this instance. It is functionally
  // const (i.e. defined in construction) but not marked const to allow for
  // default copying/assigning.
  FrameId id_{};

  // The name of the frame. Must be unique across frames from the same geometry
  // source.
  std::string name_;

  // The initial pose of frame F, measured and expressed in the parent frame P.
  Isometry3<double> X_PF_;

  // TODO(SeanCurtis-TRI): Consider whether this should be an Identifier or
  // TypeSafeIndex type.
  // The frame group to which this frame belongs.
  int frame_group_;
};

}  // namespace geometry
}  // namespace drake

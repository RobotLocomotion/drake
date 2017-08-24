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
      : name_(frame_name), X_PF_(X_PF), frame_group_(frame_group_id) {}

  const std::string& get_name() const { return name_; }
  const Isometry3<double>& get_pose() const { return X_PF_; }
  int get_frame_group() const { return frame_group_; }

 private:
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

#pragma once

#include <string>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"

namespace drake {
namespace geometry {
namespace internal {

/** This class represents the internal representation of a GeometryFrame. It
 includes the user-specified data (name and frame group), excludes the pose
 data, and then includes topology data.

 It is not intended to be used outside of the geometry library. To instantiate
 frames in SceneGraph, use the drake::geometry::GeometryFrame class in
 conjunction with the SceneGraph::RegisterFrame() methods. */
class InternalFrame {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalFrame)

  /** Default constructor. The parent identifier and pose index will be
   invalid. */
  InternalFrame();

  /** Full constructor.
   @param source_id     The identifier of the source this belongs to.
   @param frame_id      The identifier of _this_ frame.
   @param name          The name of the frame.
   @param frame_group   The frame's frame group membership.
   @param pose_index    The position in the pose vector of this frame's last
                        known pose.
   @param parent_id     The id of the parent frame.
   @param clique        The clique that will be used to prevent self-collision
                        among geometries rigidly affixed to this frame.
   */
  InternalFrame(SourceId source_id, FrameId frame_id, const std::string &name,
                int frame_group, PoseIndex pose_index, FrameId parent_id,
                int clique);

  /** Compares two %InternalFrame instances for "equality". Two internal frames
   are considered equal if they have the same frame identifier. */
  bool operator==(const InternalFrame &other) const;

  /** Compares two %InternalFrame instances for inequality. See operator==()
   for the definition of equality. */
  bool operator!=(const InternalFrame &other) const;

  /** @name      Frame properties    */
  //@{

  /** Returns the source id that registered the frame.  */
  SourceId source_id() const { return source_id_; }

  /** Returns the globally unique identifier for this frame.  */
  FrameId id() const { return id_; }

  /** Returns the name of this frame.  */
  const std::string& name() const { return name_; }

  /** Returns the frame group of this frame. It is an externally defined integer
   value that can be used to create relationships between frames; it has no
   internal significance or dependencies.  */
  int frame_group() const { return frame_group_; }

  /** Returns the pose index of this frame in the full scene graph.  */
  PoseIndex pose_index() const { return pose_index_; }

  /** Returns the clique associated with this frame. */
  int clique() const { return clique_; }

  //@}

  /** @name     Scene Graph topology    */
  //@{

  /** Returns true if this frame has the given id as its parent.  */
  bool has_parent(FrameId parent) const { return parent_id_ == parent; }

  /** Returns true if this frame is the world frame.  */
  bool is_world() const { return id_ == world_frame_id(); }

  /** Returns the id of this frame's parent frame. If this is the world frame,
   it returns its own id.  */
  FrameId parent_frame_id() const { return parent_id_; }

  /** Returns a list of ids of the frames that have *this* frame as a parent
   frame.  */
  const std::unordered_set<FrameId>& child_frames() const {
    return child_frames_;
  }

  /** Returns a list of ids of the geometries that are *directly* attached to
   this frame. It does *not* include geometries that are attached to child
   frames of this frame.  */
  const std::unordered_set<GeometryId>& child_geometries() const {
    return child_geometries_;
  }

  /** The number of total geometries attached to this frame. It should be true
   that `this->num_child_geometries() == this->child_geometries().size()`.
   To determine the number of geometries attached to this frame *by geometry
   role*, use the GeometryState::GetNumFrameGeometriesByRole() method.  */
  int num_child_geometries() const {
    return static_cast<int>(child_geometries_.size());
  }

  /** Returns true if the given `frame_id` is a known child of this frame. This
   is not coordinated. If this function returns true, it does not guarantee that
   the frame referenced by `frame_id` would report this frame as parent.  */
  bool has_child(FrameId frame_id) const {
    return child_frames_.count(frame_id) > 0;
  }

  /** Adds the given `frame_id` to the children of this frame. This does *not*
   guarantee that the frame referenced by `frame_id` will correctly report its
   parent as this frame.  */
  void add_child(FrameId frame_id) {
    child_frames_.insert(frame_id);
  }

  /** Returns if the given `geometry_id` is rigidly affixed to this frame. It
   does not guarantee that the geometry referenced by `geometry_id` would
   report this frame as its parent.  */
  bool has_child(GeometryId geometry_id) const {
    return child_geometries_.count(geometry_id) > 0;
  }

  /** Adds the given `geometry_id` to the set of rigidly affixed child
   geometries of this frame. */
  void add_child(GeometryId geometry_id) {
    child_geometries_.insert(geometry_id);
  }

  /** The identifier used for identifying the world frame in all instances of
   SceneGraph.  */
  static FrameId world_frame_id() { return kWorldFrame; }

 private:
  // The identifier of the source, to which this frame belongs.
  SourceId source_id_;

  // The identifier of this frame.
  FrameId id_;

  // The name of the frame. Must be unique across frames from the same
  // geometry source.
  std::string name_;

  // TODO(SeanCurtis-TRI): Consider whether this should be an Identifier or
  // TypeSafeIndex type.
  // The frame group to which this frame belongs.
  int frame_group_{0};

  // The index in the pose vector where this frame's pose lives.
  PoseIndex pose_index_{};

  // The clique used to prevent self-collision among the geometries affixed to
  // this frame.
  int clique_{};

  // The identifier of this frame's parent frame.
  FrameId parent_id_;

  // The identifiers of the frames, who have this frame as parent.
  std::unordered_set<FrameId> child_frames_;

  // The identifiers for the geometries that are rigidly affixed to this frame.
  // This includes geometries that were hung directly on the frame and those
  // that were hung on geometries that were already rigidly affixed.
  // It does *not* include geometries hung on child frames.
  std::unordered_set<GeometryId> child_geometries_;

  // The frame identifier of the world frame.
  static const FrameId kWorldFrame;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

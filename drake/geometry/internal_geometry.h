#pragma once

#include <string>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace internal {

/** This class represents the internal representation of registered geometry
 and its topological relationships. */
class InternalGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometry)

  /** Default constructor. The ids will be invalid, with no children, and no
   parent geometry identifier. */
  InternalGeometry() {}

  /** Full constructor.
   @param frame_id      The identifier of the frame this belongs to.
   @param geometry_id   The identifier for _this_ geometry.
   @param parent_id     The optional id of the parent geometry.
   */
  InternalGeometry(FrameId frame_id, GeometryId geometry_id,
                   const optional<GeometryId>& parent_id = {}) :
      frame_id_(frame_id),
      id_(geometry_id),
      parent_id_(parent_id) {}

  /** Compares two %InternalGeometry instances for "equality". Two internal
   frames are considered equal if they have the same geometry identifier. */
  bool operator==(const InternalGeometry &other) const {
    return id_ == other.id_;
  }

  /** Compares two %InternalGeometry instances for inequality. See operator==()
   for the definition of equality. */
  bool operator!=(const InternalGeometry &other) const {
    return !(*this == other);
  }

  FrameId get_frame_id() const { return frame_id_; }
  GeometryId get_id() const { return id_; }
  optional<GeometryId> get_parent_id() const { return parent_id_; }

  /** Returns true if this geometry has a geometry parent and the parent has the
   given `geometry_id`. */
  bool is_child_of_geometry(GeometryId geometry_id) const {
    return parent_id_ && *parent_id_ == geometry_id;
  }

  /** Returns true if the geometry is affixed to the frame with the given
   `frame_id`. */
  bool is_child_of_frame(FrameId frame_id) const {
    return frame_id == frame_id_;
  }

  const std::unordered_set<GeometryId>& get_child_geometry_ids() const {
    return child_geometry_ids_;
  }
  std::unordered_set<GeometryId>* get_mutable_child_geometry_ids() {
    return &child_geometry_ids_;
  }

  /** Returns true if this geometry has a child geometry with the given
   `geometry_id`. */
  bool has_child(GeometryId geometry_id) const {
    return child_geometry_ids_.find(geometry_id) != child_geometry_ids_.end();
  }

  /** Adds a geometry with the given `geometry_id` to this geometry's set of
   children. */
  void add_child(GeometryId geometry_id) {
    child_geometry_ids_.insert(geometry_id);
  }

  /** Removes the given `geometry_id` from this geometry's set of children. If
   the id is not in the set, nothing changes. */
  void remove_child(GeometryId geometry_id) {
    child_geometry_ids_.erase(geometry_id);
  }

 private:
  // The identifier of the frame to which this geometry belongs.
  FrameId frame_id_;

  // The identifier for this frame.
  GeometryId id_;

  // The identifier for this frame's parent frame.
  optional<GeometryId> parent_id_;

  // The identifiers for the geometry hung on this frame.
  std::unordered_set<GeometryId> child_geometry_ids_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

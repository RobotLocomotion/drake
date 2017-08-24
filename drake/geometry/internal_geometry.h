#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Include additional user-specified payload metadata when
// added to the declaration of GeometryInstance.
/** Base class for the internal representation of registered geometry. It
 includes the data common to both anchored and dynamic geometry. */
class InternalGeometryBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometryBase)

  /** Default constructor. The geometry id will be invalid, the shape will
   be nullptr, and the pose will be uninitialized. */
  InternalGeometryBase() {}

  /** Full constructor.
   @param shape         The shape specification for this instance.
   @param geometry_id   The identifier for _this_ geometry.
   @param X_PG          The pose of the geometry G in the parent frame P. */
  InternalGeometryBase(std::unique_ptr<Shape> shape, GeometryId geometry_id,
                       const Isometry3<double>& X_PG)
      : shape_spec_(std::move(shape)),
        id_(geometry_id),
        X_PG_(X_PG) {}

  /** Compares two %InternalGeometryBase instances for "equality". Two internal
   geometries are considered equal if they have the same geometry identifier. */
  bool operator==(const InternalGeometryBase &other) const {
    return id_ == other.id_;
  }

  /** Compares two %InternalGeometry instances for inequality. See operator==()
   for the definition of equality. */
  bool operator!=(const InternalGeometryBase &other) const {
    return !(*this == other);
  }

  const Shape& get_shape() const { return *shape_spec_; }
  GeometryId get_id() const { return id_; }
  const Isometry3<double>& get_pose_in_parent() const { return X_PG_; }

 private:
  // The specification for this instance's shape.
  copyable_unique_ptr<Shape> shape_spec_;

  // The identifier for this frame.
  GeometryId id_;

  // The pose of this geometry in the parent frame. The parent may be a frame or
  // another registered geometry.
  Isometry3<double> X_PG_;
};

/** This class represents the internal representation of registered _dynamic_
 geometry. It includes the user-specified meta data (e.g., name) and internal
 topology representations. */
class InternalGeometry : public InternalGeometryBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometry)

  /** Default constructor. The parent and frame ids will be invalid as well as
   the state documented in InternalGeometryBase(). */
  InternalGeometry();

  /** Full constructor.
   @param shape         The shape specification for this instance.
   @param frame_id      The identifier of the frame this belongs to.
   @param geometry_id   The identifier for _this_ geometry.
   @param X_PG          The pose of the geometry G in the parent frame P. The
                        parent may be a frame, or another registered geometry.
   @param engine_index  The position in the geometry engine of this geometry.
   @param parent_id     The optional id of the parent geometry. */
  InternalGeometry(std::unique_ptr<Shape> shape, FrameId frame_id,
                   GeometryId geometry_id, const Isometry3<double>& X_PG,
                   GeometryIndex engine_index,
                   const optional<GeometryId>& parent_id = {});

  FrameId get_frame_id() const { return frame_id_; }
  optional<GeometryId> get_parent_id() const { return parent_id_; }
  void set_parent_id(GeometryId id) { parent_id_ = id; }
  GeometryIndex get_engine_index() const { return engine_index_; }
  void set_engine_index(GeometryIndex index) { engine_index_ = index; }

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

  // The index of the geometry in the engine.
  GeometryIndex engine_index_;

  // The identifier for this frame's parent frame.
  optional<GeometryId> parent_id_;

  // The identifiers for the geometry hung on this frame.
  std::unordered_set<GeometryId> child_geometry_ids_;
};

/** This class represents the internal representation of registered _anchored_
 geometry. It includes the user-specified meta data (e.g., name) and internal
 topology representations. */
class InternalAnchoredGeometry : public InternalGeometryBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalAnchoredGeometry)

  /** Default constructor. State will be as documented in
   InternalGeometryBase(). */
  InternalAnchoredGeometry();

  /** Full constructor.
   @param shape         The shape specification for this instance.
   @param geometry_id   The identifier for _this_ geometry.
   @param X_WG          The pose of the geometry G in the world frame W.
   @param engine_index  The position in the geometry engine of this geometry. */
  InternalAnchoredGeometry(std::unique_ptr<Shape> shape, GeometryId geometry_id,
                           const Isometry3<double>& X_WG,
                           AnchoredGeometryIndex engine_index);

  AnchoredGeometryIndex get_engine_index() const { return engine_index_; }
  void set_engine_index(AnchoredGeometryIndex index) { engine_index_ = index; }

 private:
  // The index of the geometry in the engine.
  AnchoredGeometryIndex engine_index_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

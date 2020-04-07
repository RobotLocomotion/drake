#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/** The internal representation of the fixed portion of the scene graph
 geometry. This includes those aspects that do *not* depend on computed values.
 It includes things like the topology (which frame is a geometry attached to),
 internal bookkeeping (how do we identify a proximity-engine representation of
 the registered geometry), its name, and its *declared* geometry.  */
class InternalGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InternalGeometry)

  // TODO(SeanCurtis-TRI): Is this strictly required? Typically, I have this to
  // be compatible with STL structures that need to default construct. Confirm
  // and document if such is the case.
  /** Default constructor. The geometry id will be invalid, the shape will
   be nullptr, and the pose will be uninitialized.  */
  InternalGeometry() {}

  /** Constructs the internal geometry without any assigned roles defined as an
   *immediate* child of the frame. Therefore, it is assumed that X_FG = X_PG.
   @param source_id     The id for the source that registered this geometry.
   @param shape         The shape specification for this instance.
   @param frame_id      The id of the frame this belongs to.
   @param geometry_id   The identifier for _this_ geometry.
   @param name          The name of the geometry.
   @param X_FG          The pose of the geometry G in the parent frame F.  */
  InternalGeometry(SourceId source_id, std::unique_ptr<Shape> shape,
                   FrameId frame_id, GeometryId geometry_id, std::string name,
                   math::RigidTransform<double> X_FG);

  /** Compares two %InternalGeometry instances for "equality". Two internal
   geometries are considered equal if they have the same geometry identifier.
   */
  bool operator==(const InternalGeometry &other) const {
    return id_ == other.id_;
  }

  /** Compares two %InternalGeometry instances for inequality. See operator==()
   for the definition of equality.  */
  bool operator!=(const InternalGeometry &other) const {
    return !(*this == other);
  }

  /**  @name     Geometry properties  */
  //@{

  /** Returns the shape specification for this geometry.  */
  const Shape& shape() const { return *shape_spec_; }

  /** Returns the globally unique identifier for this geometry.  */
  GeometryId id() const { return id_; }

  /** Returns the name of this geometry.  */
  const std::string& name() const { return name_; }

  /** Returns the source id that registered the geometry.  */
  SourceId source_id() const { return source_id_; }

  /** Returns true if this geometry belongs to the source with the given id.  */
  bool belongs_to_source(SourceId id) const { return source_id_ == id; }

  //@}

  /** @name     Scene Graph topology    */
  //@{

  /** Reports the frame this internal geometry is rigidly attached to.  */
  FrameId frame_id() const { return frame_id_; }

  /** Returns true if the geometry is affixed to the frame with the given
   `frame_id`.  */
  bool is_child_of_frame(FrameId frame_id) const {
    return frame_id == frame_id_;
  }

  /** Returns the pose of this geometry in the declared *parent* frame -- note
   if this geometry was registered as a child of another geometry it will *not*
   be the same as X_FG().  */
  const math::RigidTransform<double>& X_PG() const { return X_PG_; }

  /** Returns the pose of this geometry in the frame to which it is ultimately
   rigidly attached. This is in contrast to X_PG().  */
  const math::RigidTransform<double>& X_FG() const { return X_FG_; }

  // TODO(SeanCurtis-TRI): Determine if tracking this parent geometry is
  // necessary for now or if that only exists to facilitate removal later on.
  /** Returns the declared parent geometry (if one exists).  */
  std::optional<GeometryId> parent_id() const { return parent_geometry_id_; }

  /** Sets this geometry to have *another* geometry as parent. In this case,
   the pose set in the constructor is assumed to be X_PG and the X_FG value must
   be updated.
   @param id    The id of the parent geometry.
   @param X_FG  The new value for X_FG (assuming the constructed value is to be
                interpreted as X_PG).  */
  void set_geometry_parent(GeometryId id, math::RigidTransform<double> X_FG) {
    parent_geometry_id_ = id;
    X_FG_ = std::move(X_FG);
  }

  /** Returns true if this geometry has a geometry parent and the parent has the
   given `geometry_id`.  */
  bool is_child_of_geometry(GeometryId geometry_id) const {
    return parent_geometry_id_ && *parent_geometry_id_ == geometry_id;
  }

  /** Returns a list of identifiers of geometries that have *this* geometry as
   a parent. In other words, for an internal geometry `g`, `g.index()`
   appears in this list iff `this->is_child_of_geometry(g.index())` returns
   true.  */
  const std::unordered_set<GeometryId>& child_geometry_ids() const {
    return child_geometry_ids_;
  }

  /** Returns true if this geometry has a child geometry with the given
   `geometry_id`.  */
  bool has_child(GeometryId geometry_id) const {
    return child_geometry_ids_.find(geometry_id) != child_geometry_ids_.end();
  }

  /** Adds a geometry with the given `geometry_id` to this geometry's set of
   children.  */
  void add_child(GeometryId geometry_id) {
    child_geometry_ids_.insert(geometry_id);
  }

  /** Removes the given `geometry_id` from the set of geometries that this frame
   considers to be children. If the id is not in the set of children, nothing
   happens.  */
  void remove_child(GeometryId geometry_id) {
    DRAKE_ASSERT(child_geometry_ids_.count(geometry_id) > 0);
    child_geometry_ids_.erase(geometry_id);
  }

  /** Returns true if the geometry is *not* attached to the world frame -- or,
   in other words, it *is* attached to a movable frame.  */
  bool is_dynamic() const {
    // NOTE: If I allow non-dynamic frames welded to the world, this will not
    // be sufficient.
    return frame_id_ != InternalFrame::world_frame_id();
  }

  //@}

  /** @name   Role management

   These methods determine what the internal geometry *knows* about its role.
   Updating a geometry's knowledge is not the same as actually applying that
   role. It is the job of GeometryState to make sure that an individual
   geometry's understanding of its role is kept in sync with the corresponding
   engine's understanding as well.  */
  //@{

  /** Assigns a proximity role to this geometry, replacing any properties that
   were previously assigned.  */
  void SetRole(ProximityProperties properties) {
    proximity_props_ = std::move(properties);
  }

  /** Assigns a illustration role to this geometry. Fails if it has already been
   assigned.  */
  void SetRole(IllustrationProperties properties) {
    if (illustration_props_) {
      throw std::logic_error("Geometry already has illustration role assigned");
    }
    illustration_props_ = std::move(properties);
  }

  /** Assigns a perception role to this geometry. Throws if the geometry has
   already had perception properties assigned.  */
  void SetRole(PerceptionProperties properties) {
    if (perception_props_) {
      throw std::logic_error("Geometry already has perception role assigned");
    }
    perception_props_ = std::move(properties);
  }

  /** Reports if the geometry has the indicated `role`.  */
  bool has_role(Role role) const;

  /** Reports if the geometry has a proximity role.  */
  bool has_proximity_role() const { return proximity_props_ != std::nullopt; }

  /** Reports if the geometry has a illustration role.  */
  bool has_illustration_role() const {
    return illustration_props_ != std::nullopt;
  }

  /** Reports if the geometry has a perception role. */
  bool has_perception_role() const { return perception_props_ != std::nullopt; }

  /** Returns a pointer to the geometry's proximity properties (if they are
   defined. Nullptr otherwise.  */
  const ProximityProperties* proximity_properties() const {
    if (proximity_props_) return &*proximity_props_;
    return nullptr;
  }

  /** Returns a pointer to the geometry's illustration properties (if they are
   defined. Nullptr otherwise.  */
  const IllustrationProperties* illustration_properties() const {
    if (illustration_props_) return &*illustration_props_;
    return nullptr;
  }

  /** Returns a pointer to the geometry's perception properties, or nullptr if
   not defined.  */
  const PerceptionProperties* perception_properties() const {
    if (perception_props_) return &*perception_props_;
    return nullptr;
  }

  /** Removes the proximity role assigned to this geometry -- if there was
   no proximity role previously, this has no effect.  */
  void RemoveProximityRole() {
    proximity_props_ = std::nullopt;
  }

  /** Removes the illustration role assigned to this geometry -- if there was
   no illustration role previously, this has no effect.  */
  void RemoveIllustrationRole() {
    illustration_props_ = std::nullopt;
  }

  /** Removes the perception role assigned to this geometry -- if there was
   no perception role previously, this has no effect.  */
  void RemovePerceptionRole() {
    perception_props_ = std::nullopt;
  }

  //@}

 private:
  // The specification for this instance's shape.
  copyable_unique_ptr<Shape> shape_spec_;

  // The identifier for this frame.
  GeometryId id_;

  // The name of the geometry. Must be unique among geometries attached to the
  // same frame.
  std::string name_;

  // The source id that registered the geometry.
  SourceId source_id_;

  // The identifier of the frame to which this geometry belongs.
  FrameId frame_id_;

  // The pose of this geometry in the registered parent frame. The parent may be
  // a frame or another registered geometry.
  math::RigidTransform<double> X_PG_;

  // The pose of this geometry in the ultimate frame to which this geometry is
  // rigidly affixed. If there is no parent geometry, X_PG_ == X_FG_.
  math::RigidTransform<double> X_FG_;

  // The identifier for this frame's parent frame.
  std::optional<GeometryId> parent_geometry_id_{};

  // The identifiers for the geometry hung on this frame.
  std::unordered_set<GeometryId> child_geometry_ids_;

  // TODO(SeanCurtis-TRI): Consider introducing a mechanism where these are
  // defined at the frame level, and all child geometries inherit.

  // The optional property sets tied to the roles that the geometry plays.
  std::optional<ProximityProperties> proximity_props_{};
  std::optional<IllustrationProperties> illustration_props_{};
  std::optional<PerceptionProperties> perception_props_{};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

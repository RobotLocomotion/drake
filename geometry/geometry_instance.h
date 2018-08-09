#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/visual_material.h"

namespace drake {
namespace geometry {

/** A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent "frame" P), material information, and an
 opaque collection of metadata. The parent frame can be a registered frame or
 another registered geometry.

 Every %GeometryInstance must be named. The naming convention mirrors that of
 valid names in SDF files. Specifically, any user-specified name will have
 all leading and trailing space and tab characters trimmed off. The trimmed name
 will have to satisfy the following requirements:
   - cannot be empty, and
   - the name must be unique in the scope of its frame. In other words, two
     GeometryInstances can both be called "collision" as long as they are
     affixed to different frames.
 If valid, the trimmed name will be assigned to the instance.
 <!-- Note to developers: The sdf requirements for naming are captured in a
 unit test in `scene_graph_parser_detail_test.cc. See test
 VisualGeometryNameRequirements and keep those tests and this list in sync. -->

 Names *can* have internal whitespace (e.g., "my frame name").

 <!-- TODO(SeanCurtis-TRI): Refine the scope to just the *role* within the
 frame. -->
 */
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  /** Constructor with default visual material (see VisualMaterial default
   constructor for details on what that color is).
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance.
   @param name   The name of the geometry (must satisfy the name requirements).
   */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape,
                   std::string name);

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance.
   @param name   The name of the geometry (must satisfy the nam requirements).
   @param vis_material The visual material to apply to this geometry.  */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape,
                   std::string name,
                   const VisualMaterial& vis_material);

  /** Returns the globally unique id for this geometry specification. Every
   instantiation of %GeometryInstance will contain a unique id value. The id
   value is preserved across copies. After successfully registering this
   %GeometryInstance, this id will serve as the identifier for the registered
   representation as well. */
  GeometryId id() const { return id_; }

  const Isometry3<double>& pose() const { return X_PG_; }
  void set_pose(const Isometry3<double>& X_PG) { X_PG_ = X_PG; }

  const Shape& shape() const {
    DRAKE_DEMAND(shape_ != nullptr);
    return *shape_;
  }

  /** Releases the shape from the instance. */
  std::unique_ptr<Shape> release_shape() { return std::move(shape_); }

  const VisualMaterial& visual_material() const { return visual_material_; }

  const std::string& name() const { return name_; }

 private:
  // The *globally* unique identifier for this instance. It is functionally
  // const (i.e. defined in construction) but not marked const to allow for
  // default copying/assigning.
  GeometryId id_{};

  // The pose of the geometry relative to the parent frame it hangs on.
  Isometry3<double> X_PG_;

  // The shape associated with this instance.
  copyable_unique_ptr<Shape> shape_;

  // The name of the geometry instance.
  std::string name_;

  // The "rendering" material -- e.g., OpenGl contexts and the like.
  VisualMaterial visual_material_;
};
}  // namespace geometry
}  // namespace drake

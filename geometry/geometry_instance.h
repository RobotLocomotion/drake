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
   - the name should be unique in the scope of its frame and role. For example,
     two GeometryInstances can both be called "ball" as long as they are
     affixed to different frames or if one is a collision geometry and the
     other is a visual geometry. This requirement is not *currently* enforced
     but will be enforced in the future.
     <!-- TODO(SeanCurtis-TRI): When geometry roles lands, change this to
     indicate that this is enforced. -->
 If valid, the trimmed name will be assigned to the instance.

 Names *can* have internal whitespace (e.g., "my geometry name").

 @anchor canonicalized_geometry_names
 <h3>Canonicalized names</h3>

 The silent transformation of a user-defined name to canonical name mirrors that
 of specifying geometry names in an SDF file. Consider the following SDF
 snippet:

 @code{xml}
   ...
   <visual name="  visual">
     <geometry>
       <sphere>
         <radius>1.0</radius>
       </sphere>
     </geometry>
   </visual>
   ...
 @endcode

 The name has two leading whitespace characters. The parsing process will
 consider this name as equivalent to "visual" and tests for uniqueness and
 non-emptiness will be applied to that trimmed result. The following code has
 an analogous effect:

 ```
 scene_graph->RegisterGeometry(
    source_id, frame_id,
    make_unique<GeometryInstance>(pose, make_unique<Sphere>(1.0), "  visual"));
 ```

 The specified name includes leading whitespace. That name will be trimmed and
 the *result* will be stored in the %GeometryInstance (to be later validated by
 SceneGraph as part of geometry registration). Querying the instance of its name
 will return this *canonicalized* name.

 <!-- Note to developers: The sdf requirements for naming are captured in a
 unit test in `scene_graph_parser_detail_test.cc. See test
 VisualGeometryNameRequirements and keep those tests and this list in sync. -->
 */
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  /** Constructor with default visual material (see VisualMaterial default
   constructor for details on what that color is).
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance.
   @param name   The name of the geometry (must satisfy the name requirements).
   @throws std::logic_error if the canonicalized version of `name` is empty.
   */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape,
                   const std::string& name);

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance.
   @param name   The name of the geometry (must satisfy the name requirements).
   @param vis_material The visual material to apply to this geometry.
   @throws std::logic_error if the canonicalized version of `name` is empty.  */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape,
                   const std::string& name,
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

  /** Returns the *canonicalized* name for the instance. */
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

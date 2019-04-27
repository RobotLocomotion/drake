
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** The engine for performing rasterization operations on geometry. This
 includes rgb images and depth images. The coordinate system of
 %RenderEngine's viewpoint `R` is `X-right`, `Y-down` and `Z-forward`
 with respect to the rendered images.

 Output image format:
   - RGB (ImageRgba8U) : the RGB image has four channels in the following
     order: red, green, blue, and alpha. Each channel is represented by
     a uint8_t.

   - Depth (ImageDepth32F) : the depth image has a depth channel represented
     by a float. For a point in space `P`, the value stored in the depth
     channel holds *the Z-component of the position vector `p_RP`.*
     Note that this is different from the range data used by laser
     range finders (like that provided by DepthSensor) in which the depth
     value represents the distance from the sensor origin to the object's
     surface.  */
class RenderEngine : public ShapeReifier {
 public:
  RenderEngine() = default;

  virtual ~RenderEngine() = default;

  /** Clones the render engine -- making the RenderEngine compatible with
   copyable_unique_ptr.  */
  std::unique_ptr<RenderEngine> Clone() const;

  /** Registers a shape specification and returns the optional index of the
   corresponding render geometry. The geometry can be uniquely referenced in
   this engine (and copies of this engine) by its geometry index. The renderer
   is allowed to examine the given `properties` and choose to _not_ register
   the geometry.

   @param index          The geometry index of the shape to register.
   @param shape          The shape specification to add to the render engine.
   @param properties     The perception properties provided for this geometry.
   @param X_WG           The pose of the geometry relative to the world frame W.
   @param needs_updates  If true, the geometry's pose will be updated via
                         UpdatePoses().
   @returns A unique index for the resultant render geometry (nullopt if not
            registered).
   @throws std::runtime_error if the shape is an unsupported type.  */
  optional<RenderIndex> RegisterVisual(
      GeometryIndex index,
      const Shape& shape, const PerceptionProperties& properties,
      const math::RigidTransformd& X_WG, bool needs_updates = true);

  /** Removes the geometry indicated by the given `index` from the engine.
   It may move another geometry into that index value to maintain a contiguous
   block of indices. If it does so, it returns the internal geometry index of
   the geometry that got moved into the `index` position.
   @param index  The _render_ index of the geometry to remove.
   @returns The _geometry_ index of the geometry that _now_ maps to the _render_
            `index` if the `index` was recycled.
   @throws std::logic_error if the index is invalid.  */
  optional<GeometryIndex> RemoveGeometry(RenderIndex index);

  /** Updates the poses of all geometries marked as "needing update" (see
 RegisterVisual()).

 @param  X_WGs   The poses of *all* geometries in SceneGraph (measured and
                 expressed in the world frame). The pose for a geometry is
                 accessed by that geometry's GeometryIndex.  */
  template <typename T>
  void UpdatePoses(const std::vector<math::RigidTransform<T>>& X_WGs) {
    for (auto pair : update_indices_) {
      RenderIndex render_index = pair.first;
      GeometryIndex geometry_index = pair.second;
      DoUpdateVisualPose(render_index,
                         geometry::internal::convert(X_WGs[geometry_index]));
    }
  }

  /** Updates the renderer's viewpoint with given pose X_WR.

   @param X_WR  The pose of renderer's viewpoint in the world coordinate
                system.  */
  virtual void UpdateViewpoint(const math::RigidTransformd& X_WR) const = 0;

  /** Renders the registered geometry into the given color (rgb) image.

   @param camera                The intrinsic properties of the camera.
   @param show_window           If true, the render window will be displayed.
   @param[out] color_image_out  The rendered color image.  */
  virtual void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const = 0;

  /** Renders the registered geometry into the given depth image. In contrast to
   the other rendering operations, depth images don't have an option to display
   the window; generally, basic depth images are not readily communicative to
   humans.

   @param camera                The intrinsic properties of the camera.
   @param[out] depth_image_out  The rendered depth image.  */
  virtual void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const = 0;

 protected:
  // Allow derived classes to implement Cloning via copy-construction.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderEngine)

  /** The NVI-function for sub-classes to implement actual geometry
   registration. If the derived class chooses not to register this particular
   shape, it should return nullopt.

   A derived render engine can choose not to register geometry because, e.g., it
   doesn't have default properties. This is the primary mechanism which enables
   different renderers to use different geometries for the same frame.
   For example, a low-fidelity renderer may use simple geometry whereas a
   high-fidelity renderer would require a very detailed geometry. Both
   geometries would have PerceptionProperties, but, based on the provided
   property groups and values, one would be accepted and registered with one
   render engine implementation and the other geometry with another render
   engine.  */
  virtual optional<RenderIndex> DoRegisterVisual(
      const Shape& shape, const PerceptionProperties& properties,
      const math::RigidTransformd& X_WG) = 0;

  /** The NVI-function for updating the pose of a render geometry (identified
   by index) to the given pose X_WG.

   @param index    The index of the render geometry whose pose is being set.
   @param X_WG     The pose of the render geometry in the world frame.  */
  virtual void DoUpdateVisualPose(RenderIndex index,
                                  const math::RigidTransformd& X_WG) = 0;

  /** The NVI-function for removing the geometry at the given `index`. If the
   implementation _moves_ another geometry into this slot, the pre-move index
   for the moved geometry is returned.
   @param index  The index of the geometry to remove.
   @return  The original index of the geometry that got moved to `index`. It
            must be the case that the return value is either nullopt _or_ it
            must not be equal to `index`.  */
  virtual optional<RenderIndex> DoRemoveGeometry(RenderIndex index) = 0;

  /** The NVI-function for cloning this render engine.  */
  virtual std::unique_ptr<RenderEngine> DoClone() const = 0;

  friend class RenderEngineTester;

 private:
  // The following two maps store all registered render index values to the
  // corresponding geometry's internal index. It should be the case that the
  // keys of the two maps are disjoint and span all of the valid render index
  // values (i.e., [0, number of actors - 1]).
  // The mapping is generally necessary to facilitate updates and geometry
  // removal.

  // The mapping from render index to internal index for those geometries whose
  // poses must be updated in UpdateVisualPose.
  std::unordered_map<RenderIndex, GeometryIndex> update_indices_;

  // The mapping from render index to internal index of all other geometries.
  std::unordered_map<RenderIndex, GeometryIndex> anchored_indices_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

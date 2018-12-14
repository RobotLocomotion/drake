
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
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** The engine for performing rasterization operations on geometry. This
 includes rgb images, depth images, and, more generally, operations that can
 be performed in the OpenGL shader pipeline. The coordinate system of
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
     surface.

   - Label (ImageLabel16I) : the label image has single channel represented
     by a int16_t. The value stored in the channel holds a RenderLabel value
     which corresponds to an object class in the scene. Pixels attributable to
     no geometry contain the RenderLabel::empty_label() value. A special class
     has already been defined for "terrain": RenderLabel::terrain_label().  */
class RenderEngine : public ShapeReifier {
 public:
  // TODO(SeanCurtis-TRI): I should consider making this protected.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderEngine);

  RenderEngine() = default;

  virtual ~RenderEngine() {}

  /** Clones the render engine -- making the RenderEngine compatible with
   copyable_unique_ptr.  */
  std::unique_ptr<RenderEngine> Clone() const;

  /** Adds a flat terrain to the render engine; it renders to a default render
   color and the RenderLabel::terrain_label() label value.  */
  virtual void AddFlatTerrain() = 0;

  /** Registers a shape specification and returns the optional index of the
   corresponding render geometry. The geometry can be uniquely referenced in
   this engine (and copies of this engine) by its geometry index. The renderer
   is allowed to examine the given `properties` and choose to _not_ register
   the geometry.

   @param index          The global index of the shape to register.
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
      const Isometry3<double>& X_WG, bool needs_updates = true);

  /** Given the poses of *all* geometries in SceneGraph (measured and expressed
   in the world frame), updates the internal representations of the subset of
   geometry that are registered with _this_ renderer.  */
  template <typename T>
  void UpdatePoses(const std::vector<Isometry3<T>>& X_WG) {
    for (auto pair : update_indices_) {
      RenderIndex render_index = pair.first;
      GeometryIndex global_index = pair.second;
      DoUpdateVisualPose(internal::convert(X_WG[global_index]), render_index);
    }
  }

  /** Updates the renderer's viewpoint with given pose X_WR.

   @param X_WR  The pose of renderer's viewpoint in the world coordinate
                system.  */
  virtual void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const = 0;

  // TODO(SeanCurtis-TRI): Determine if I like the fact that the images are
  // output parameters when I have a void.  Do, I *know* the image is the right
  // size?

  /** Renders and outputs the rendered color image.

   @param camera                The intrinsic properties of the camera.
   @param[out] color_image_out  The rendered color image.
   @param show_window           If true, the render window will be displayed.
  */
  virtual void RenderColorImage(const CameraProperties& camera,
                                systems::sensors::ImageRgba8U* color_image_out,
                                bool show_window) const = 0;

  /** Renders and outputs the rendered depth image. In contrast to the other
   rendering operations, depth images don't have an option to display the
   window; generally, basic depth images are not readily communicative to
   humans.

   @param camera                The intrinsic properties of the camera.
   @param[out] depth_image_out  The rendered depth image.  */
  virtual void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const = 0;

  /** Renders and outputs the rendered label image.

   @param camera                The intrinsic properties of the camera.
   @param[out] label_image_out  The rendered label image.
   @param show_window           If true, the render window will be displayed.
  */
  virtual void RenderLabelImage(
      const CameraProperties& camera,
      systems::sensors::ImageLabel16I* label_image_out,
      bool show_window) const = 0;

 protected:
  /** The NVI-function for sub-classes to implement actual geometry
   registration. If the derived class chooses not to register this particular
   shape, it should return nullopt.

   A derived render engine can choose not to register geometry because, e.g., it
   doesn't have required properties. This is the primary mechanism which enables
   different renderers to use different geometries for the same frame.
   For example, a low-fidelity renderer may use simple geometry whereas a
   high-fidelity renderer would require a very detailed geometry. Both
   geometries would have PerceptionProperties, but, based on the provided
   properties, one would be accepted and registered with one render engine
   implementation and the other geometry with another render engine.  */
  virtual optional<RenderIndex> DoRegisterVisual(
      const Shape& shape, const PerceptionProperties& properties,
      const Isometry3<double>& X_WG) = 0;

  /** The NVI-function for updating the pose of a render geometry (identified
   by index) to the given pose X_WG.

   @param X_WG     The pose of the render geometry in the world frame.
   @param index    The index of the render geometry whose pose is being set.  */
  virtual void DoUpdateVisualPose(const Eigen::Isometry3d& X_WG,
                                RenderIndex index) = 0;

  /** The NVI-function for cloning this render engine.  */
  virtual std::unique_ptr<RenderEngine> DoClone() const = 0;

 private:
  // Mapping from this renderer's RenderIndex to the global index in SceneGraph
  // of the geometries that need updating.
  std::unordered_map<RenderIndex, GeometryIndex> update_indices_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

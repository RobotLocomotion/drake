
#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/geometry/dev/geometry_index.h"
#include "drake/geometry/dev/geometry_roles.h"
#include "drake/geometry/dev/render/camera_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace dev {
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
     has already been defined for "terrain": RenderLabel::terrain_label(). */
class RenderEngine : public ShapeReifier {
 public:
  virtual ~RenderEngine() {}

  /** Clones the render engine -- making the RenderEngine compatible with
   copyable_unique_ptr. */
  virtual std::unique_ptr<RenderEngine> Clone() const = 0;

  /** Adds a flat terrain to the render engine; it renders to a default render
   color and the RenderLabel::terrain_label() label value. */
  virtual void AddFlatTerrain() = 0;

  /** Registers a shape specification and returns the index of the corresponding
   render geometry. The geometry can be uniquely referenced in this engine (and
   copies of this engine) by its geometry index.

   @param shape       The shape specification to add to the render engine.
   @param properties  The perception properties provided for this geometry.
   @param X_FG        The pose of the geometry relative to its parent frame F.
   @returns A unique index for the resultant render geometry.
   @throws std::runtime_error if the shape is an unsupported type. */
  virtual RenderIndex RegisterVisual(
      const Shape& shape, const PerceptionProperties& properties,
      const Isometry3<double>& X_FG) = 0;

  /** Removes the visual geometry with the indicated index from `this` render
   engine. The render engine _can_ move another geometry to inherit this newly
   freed up index. If it does so, it will return the _old_ index of the geometry
   it moved.
   @pre `index` must be a valid index.  */
  virtual optional<RenderIndex> RemoveVisual(RenderIndex index) = 0;

  // TODO(SeanCurtis-TRI): I need a super-secret RegisterVisual in which the
  // index is specified.

  /** Updates the pose of a render geometry with given pose X_WG.

   @param X_WG     The pose of the render geometry in the world frame.
   @param index    The index of the render geometry whose pose is being set. */
  virtual void UpdateVisualPose(const Eigen::Isometry3d& X_WG,
                                RenderIndex index) const = 0;

  /** Updates the renderer's viewpoint with given pose X_WR.

   @param X_WR  The pose of renderer's viewpoint in the world coordinate
                system. */
  virtual void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const = 0;

  // TODO(SeanCurtis-TRI): Determine if I like the fact that the images are
  // output parameters when I have a void.  Do, I *know* the image is the right
  // size?

  /** Renders and outputs the rendered color image.

   @param camera                The intrinsic properties of the camera.
   @param[out] color_image_out  The rendered color image.
   @param show_window           If true, the render window will be displayed. */
  virtual void RenderColorImage(const CameraProperties& camera,
                                systems::sensors::ImageRgba8U* color_image_out,
                                bool show_window) const = 0;

  /** Renders and outputs the rendered depth image. In contrast to the other
   rendering operations, depth images don't have an option to display the
   window; generally, basic depth images are not readily communicative to
   humans.

   @param camera                The intrinsic properties of the camera.
   @param[out] depth_image_out  The rendered depth image. */
  virtual void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const = 0;

  /** Renders and outputs the rendered label image.

   @param camera                The intrinsic properties of the camera.
   @param[out] label_image_out  The rendered label image.
   @param show_window           If true, the render window will be displayed. */
  virtual void RenderLabelImage(
      const CameraProperties& camera,
      systems::sensors::ImageLabel16I* label_image_out,
      bool show_window) const = 0;
};

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake

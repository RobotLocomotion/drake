#pragma once

#include <limits>
#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/// An RGB-D renderer that renders RGB, depth and label images using
/// VisualElement. The coordinate system of RgbdRenderer's viewpoint `R` is
/// `X-right`, `Y-down` and `Z-forward` with respect to the rendered images.
///
/// Output image format:
///   - RGB (ImageRgba8U) : the RGB image has four channels in the following
///     order: red, green, blue and alpha. Each channel is represented by
///     a uint8_t.
///
///   - Depth (ImageDepth32F) : the depth image has a depth channel represented
///     by a float. For a point in space `P`, the value stored in the depth
///     channel holds *the Z-component of the position vector `p_RP`.*
///     Note that this is different from the range data used by laser
///     range finders (like that provided by DepthSensor) in which the depth
///     value represents the distance from the sensor origin to the object's
///     surface.
///
///   - Label (ImageLabel16I) : the label image has single channel represented
///     by a int16_t. The value stored in the channel holds a model ID which
///     corresponds to an object in the scene. For the pixels corresponding to
///     no body, namely the sky and the flat terrain, we assign Label::kNoBody
///     and Label::kFlatTerrain, respectively.
class RgbdRenderer final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdRenderer)

  /// A constructor for %RgbdRenderer.
  ///
  /// @param X_WR The initial pose of the renderer's viewpoint `R` at the world
  /// coordinate system. The `R` can be updated by calling `UpdateViewpoint`
  /// later on.
  ///
  /// @param width The width of the image to be rendered in pixels.
  ///
  /// @param height The height of the image to be rendered in pixels.
  ///
  /// @param z_near The minimum depth RgbdRenderer can output. Note that
  /// this is different from renderer's clipping range where all the objects
  /// outside the range are not rendered even in RGB image while this only
  /// affects depth image.
  ///
  /// @param z_far The maximum depth RgbdRenderer can output. Note that
  /// this is different from renderer's clipping range where all the objects
  /// outside the range are not rendered even in RGB image while this only
  /// affects depth image.
  ///
  /// @param fov_y The RgbdRenderer's vertical field of view in radians.
  ///
  /// @param show_window A flag for showing visible windows for RGB and label
  /// images.  If this is false, offscreen rendering is executed. This is
  /// useful for debugging purposes.
  RgbdRenderer(const Eigen::Isometry3d& X_WR,
               int width,
               int height,
               double z_near,
               double z_far,
               double fov_y,
               bool show_window);

  ~RgbdRenderer();

  /// Adds a flat terrain in the rendering scene.
  void AddFlatTerrain();

  /// Represents indices for visual elements.
  using VisualIndex = TypeSafeIndex<class VisualTag>;

  /// Registers a visual element to a rigid body and returns the ID of the
  /// visual element. The element is uniquely identified using `body_id` and
  /// `visual_id`.
  ///
  /// @param visual A visual element to be registered. See VisualElement for
  /// more detail.
  ///
  /// @param body_id The ID of a rigid body that you want a visual to be
  /// associated with. Note that you can associate more than one visual with a
  /// rigid body.
  ///
  /// @return visual_id A local visual ID associated with given `body_id`.
  /// `nullopt` will be returned if `visual` contains an unsupported shape.
  /// We assume `visual_id` will be used together with `body_id` when you call
  /// UpdateVisualPose() later.
  optional<VisualIndex> RegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id);

  /// Updates the pose of a visual with given pose X_WV.
  ///
  /// @param X_WV The pose of a visual in the world coordinate system.
  ///
  /// @param body_id The ID of a rigid body that the visual you want to update
  /// is associated with.
  ///
  /// @param visual_id The local ID of a visual that is associated with a rigid
  /// body and you want to update.
  ///
  // TODO(kunimatsu-tri) Remove body_id once RgbdCamera which holds RgbdRenderer
  // adapted to MultibodyTree. Both body_id and visual_id are needed here for
  // now simply because visual_ids are local IDs that you can merely use for
  // a rigid body and this mechanism is provided by RigidBodyTree which is
  // currently used by RgbdCamera.
  void UpdateVisualPose(const Eigen::Isometry3d& X_WV,
                        int body_id, VisualIndex visual_id) const;

  /// Updates renderer's viewpoint with given pose X_WR.
  ///
  /// @param X_WR The pose of renderer's viewpoint in the world coordinate
  /// system.
  void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const;

  /// Renders and outputs the rendered color image.
  ///
  /// @param color_image_out The rendered color image.
  void RenderColorImage(ImageRgba8U* color_image_out) const;

  /// Renders and outputs the rendered depth image.
  ///
  /// @param depth_image_out The rendered depth image.
  void RenderDepthImage(ImageDepth32F* depth_image_out) const;

  /// Renders and outputs the rendered label image.
  ///
  /// @param label_image_out The rendered label image.
  void RenderLabelImage(ImageLabel16I* label_image_out) const;

  /// Returns the image width.
  int width() const;

  /// Returns the image height.
  int height() const;

  /// Returns the renderer's vertical field of view.
  double fov_y() const;

  /// Returns sky's color in RGB image.
  const ColorI& get_sky_color() const;

  /// Returns flat terrain's color in RGB image.
  const ColorI& get_flat_terrain_color() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

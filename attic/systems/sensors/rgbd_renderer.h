#pragma once

#include <limits>
#include <optional>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/// Common configurations of rendering systems.
struct RenderingConfig {
  RenderingConfig(int width_in, int height_in, double fov_y_in,
                  double z_near_in, double z_far_in, bool show_window_in)
      : width(width_in),
        height(height_in),
        fov_y(fov_y_in),
        z_near(z_near_in),
        z_far(z_far_in),
        show_window(show_window_in) {}
  // TODO(eric.cousineau): Define all default values be defined here to
  // minimize duplication.
  /// The width of the image to be rendered in pixels.
  const int width{kDefaultWidth};
  /// Default value for `width`.
  static constexpr int kDefaultWidth{640};
  /// The height of the image to be rendered in pixels.
  const int height{kDefaultHeight};
  /// Default value for `height`.
  static constexpr int kDefaultHeight{480};
  /// The renderer's camera vertical field of view in radians.
  const double fov_y;
  /// The minimum depth RgbdRenderer can output. Note that this is different
  /// from renderer's clipping range where all the objects outside the range are
  /// not rendered even in RGB image while this only affects depth image.
  const double z_near;
  /// The maximum depth RgbdRenderer can output. Note that this is different
  /// from renderer's clipping range where all the objects outside the range are
  /// not rendered even in RGB image while this only affects depth image.
  const double z_far;
  /// A flag for showing visible windows for RGB and label images.  If this is
  /// false, offscreen rendering is executed. This is useful for debugging
  /// purposes.
  const bool show_window{kDefaultShowWindow};
  /// Default value for `show_window`.
  static constexpr bool kDefaultShowWindow{false};
};


/// Abstract interface of RGB-D renderers, which render RGB, depth and label
/// images using VisualElement. The coordinate system of RgbdRenderer's
/// viewpoint `R` is `X-right`, `Y-down` and `Z-forward` with respect to the
/// rendered images.
///
/// Output image format:
///
/// - RGB (ImageRgba8U) : the RGB image has four channels in the following
///   order: red, green, blue and alpha. Each channel is represented by
///   a uint8_t.
///
/// - Depth (ImageDepth32F) : the depth image has a depth channel represented
///   by a float. For a point in space `P`, the value stored in the depth
///   channel holds *the Z-component of the position vector `p_RP`.*
///   Note that this is different from the range data used by laser
///   range finders (like that provided by DepthSensor) in which the depth
///   value represents the distance from the sensor origin to the object's
///   surface.
///
/// - Label (ImageLabel16I) : the label image has single channel represented
///   by a int16_t. The value stored in the channel holds a model ID which
///   corresponds to an object in the scene. For the pixels corresponding to
///   no body, namely the sky and the flat terrain, we assign Label::kNoBody
///   and Label::kFlatTerrain, respectively.
class RgbdRenderer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdRenderer)

  // TODO(thduynguyen, kunimatsu-tri): Handle multiple viewpoints, e.g. for
  // stereo depth camera?
  /// A constructor for %RgbdRenderer.
  ///
  /// @param config Configurations of the renderer. See RenderingConfig.
  ///
  /// @param X_WC The initial pose of the renderer's unique camera viewpoint `C`
  /// in the world coordinate system. The camera pose `C` can be updated by
  /// calling `UpdateViewpoint` later on. Default value: Identity.
  RgbdRenderer(const RenderingConfig& config,
               const Eigen::Isometry3d& X_WC = Eigen::Isometry3d::Identity());

  virtual ~RgbdRenderer();

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
  std::optional<VisualIndex> RegisterVisual(
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

  /// Updates renderer's camera viewpoint with given pose X_WC.
  ///
  /// @param X_WC The pose of renderer's camera viewpoint in the world
  /// coordinate system.
  void UpdateViewpoint(const Eigen::Isometry3d& X_WC) const;

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

  /// Returns the configuration object of this renderer.
  const RenderingConfig& config() const;

  /// Returns the color palette of this renderer.
  const ColorPalette<int>& color_palette() const;

 private:
  virtual void ImplAddFlatTerrain() = 0;

  virtual std::optional<VisualIndex> ImplRegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id) = 0;

  virtual void ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV, int body_id,
                                    VisualIndex visual_id) const = 0;

  virtual void ImplUpdateViewpoint(const Eigen::Isometry3d& X_WC) const = 0;

  virtual void ImplRenderColorImage(ImageRgba8U* color_image_out) const = 0;

  virtual void ImplRenderDepthImage(ImageDepth32F* depth_image_out) const = 0;

  virtual void ImplRenderLabelImage(ImageLabel16I* label_image_out) const = 0;

  /// The common configuration needed by all implementations of this interface.
  RenderingConfig config_;

  /// The color palette for sky, terrain colors and ground truth label rendering
  /// TODO(thduynguyen, SeanCurtis-TRI): This is a world's property (colors for
  /// each object/segment) hence should be moved to GeometrSystem. That would
  /// also answer the question whether this heavy object should be a singleton.
  ColorPalette<int> color_palette_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

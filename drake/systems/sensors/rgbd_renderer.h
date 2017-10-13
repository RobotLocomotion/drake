#pragma once

#include <array>
#include <limits>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>
#if VTK_MAJOR_VERSION >= 6
#include <vtkAutoInit.h>
#endif

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

#if VTK_MAJOR_VERSION >= 6
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

namespace drake {
namespace systems {
namespace sensors {

// Register the object factories for the vtkRenderingOpenGL2 module.
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
#if VTK_MAJOR_VERSION >= 6
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
#endif
  }
};

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
class RgbdRenderer final : private ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdRenderer)

  /// Set of constants used to represent invalid depth values.
  /// Note that if a depth is not measurable, NaN will be set.
  class InvalidDepth {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InvalidDepth)
    /// The depth value when the max sensing range is exceeded.
    static constexpr float kTooFar{std::numeric_limits<float>::infinity()};

    /// The depth value when the min sensing range is violated because the
    /// object being sensed is too close. Note that this
    /// <a href="http://www.ros.org/reps/rep-0117.html">differs from ROS</a>,
    /// which uses negative infinity in this scenario. Drake uses zero because
    /// it results in less devastating bugs when users fail to check for the
    /// lower limit being hit and using negative infinity does not prevent users
    /// from writing bad code.
    static constexpr float kTooClose{0.f};
  };

  /// Set of labels used for label image.
  class Label {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Label)
    /// The label used for pixels correspond to nothing.
    static constexpr int16_t kNoBody{std::numeric_limits<int16_t>::max()};
    /// The label used for pixels correspond to the flat terrain.
    static constexpr int16_t kFlatTerrain{
      std::numeric_limits<int16_t>::max() - 1};
  };

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

  ~RgbdRenderer() = default;

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
  int width() const { return width_; }

  /// Returns the image height.
  int height() const { return height_; }

  /// Returns the renderer's vertical field of view.
  double fov_y() const { return fov_y_; }

  /// Returns sky's color in RGB image.
  const ColorI& get_sky_color() const {
    return color_palette_.get_sky_color(); }

  /// Returns flat terrain's color in RGB image.
  const ColorI& get_flat_terrain_color() const {
    return color_palette_.get_terrain_color(); }

 private:
  float CheckRangeAndConvertToMeters(float z_buffer_value) const;

  const int width_;
  const int height_;
  const double fov_y_;
  const double z_near_;
  const double z_far_;
  const ColorPalette color_palette_;

  vtkNew<vtkActor> terrain_actor_;
  // An array of maps which take pairs of a body index in RBT and a vector of
  // vtkSmartPointer to vtkActor. The each vtkActor corresponds to an visual
  // element specified in SDF / URDF. The first element of this array is for
  // color and depth rendering and the second is for label image rendering.
  // TODO(kunimatsu-tri) Make this more straight forward for the readability.
  std::array<std::map<int, std::vector<vtkSmartPointer<vtkActor>>>, 2>
      id_object_maps_;
  vtkNew<vtkRenderer> color_depth_renderer_;
  vtkNew<vtkRenderer> label_renderer_;
  vtkNew<vtkRenderWindow> color_depth_render_window_;
  vtkNew<vtkRenderWindow> label_render_window_;
  vtkNew<vtkWindowToImageFilter> color_filter_;
  vtkNew<vtkWindowToImageFilter> depth_filter_;
  vtkNew<vtkWindowToImageFilter> label_filter_;
  vtkNew<vtkImageExport> color_exporter_;
  vtkNew<vtkImageExport> depth_exporter_;
  vtkNew<vtkImageExport> label_exporter_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

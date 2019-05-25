
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
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** The engine for performing rasterization operations on geometry. This
 includes rgb images and depth images. The coordinate system of
 %RenderEngine's viewpoint `R` is `X-right`, `Y-down` and `Z-forward`
 with respect to the rendered images.

 <h3>Output image format</h3>

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
     by an int16_t. The value stored in the channel holds a RenderLabel value
     which corresponds to an object class in the scene or an "empty" pixel (see
     RenderLabel for more details).

 @anchor render_engine_default_label
 <h3>RenderLabels, registering geometry, and derived classes</h3>

 By convention, when registering a geometry, the provided properties should
 contain no more than one RenderLabel instance, and that should be the
 `(label, id)` property. %RenderEngine provides the notion of a
 _default render label_ that will be applied where no `(label, id)` RenderLabel
 property is found.  This default value can be one of two values:
 RenderLabel::kDontCare or RenderLabel::kUnspecified. The choice of default
 RenderLabel can be made at construction and it affects registration behavior
 when the `(label, id)` property is absent:

   - RenderLabel::kUnspecified: throws an exception.
   - RenderLabel::kDontCare: the geometry will be included in label images as
     the generic, non-distinguishing label.

 Choosing RenderLabel::kUnspecified is best in a system that wants explicit
 feedback and strict enforcement on a policy of strict label enforcement --
 everything should receive a meaningful label. The choice of
 RenderLabel::kDontCare is best for a less strict system in which only some
 subset of geometry need be explicitly specified.

 Derived classes configure their _de facto_ default RenderLabel value, or
 a user-configured default value, at construction, subject to the requirements
 outlined above.

 Derived classes should not access the `(label, id)` property directly.
 %RenderEngine provides a method to safely extract a RenderLabel value from
 the PerceptionProperties, taking into account the configured default value and
 the documented @ref reserved_render_label "RenderLabel semantics"; see
 GetRenderLabelOrThrow().  */
class RenderEngine : public ShapeReifier {
 public:
  /** Constructs a %RenderEngine with the given default render label. The
   default render label is applied to geometries that have not otherwise
   specified a (label, id) property. The value _must_ be either
   RenderLabel::kUnspecified or RenderLabel::kDontCare. (See
   @ref render_engine_default_label "this section" for more details.)

   @throws std::logic_error if the default render label is not one of the two
                            allowed labels.  */
  explicit RenderEngine(
      const RenderLabel& default_label = RenderLabel::kUnspecified)
      : default_render_label_(default_label) {
    if (default_render_label_ != RenderLabel::kUnspecified &&
        default_render_label_ != RenderLabel::kDontCare) {
      throw std::logic_error(
          "RenderEngine's default render label must be either 'kUnspecified' "
          "or 'kDontCare'");
    }
  }

  virtual ~RenderEngine() = default;

  /** Clones the render engine -- making the %RenderEngine compatible with
   copyable_unique_ptr.  */
  std::unique_ptr<RenderEngine> Clone() const;

  /** Registers a shape specification and returns the optional index of the
   corresponding render geometry. The geometry can be uniquely referenced in
   this engine (and copies of this engine) by its geometry index. The renderer
   is allowed to examine the given `properties` and choose to _not_ register
   the geometry.

   Typically, derived classes will attempt to validate the RenderLabel value
   stored in the `(label, id)` property (or its configured default value if
   no such property exists). In that case, attempting to assign
   RenderLabel::kEmpty or RenderLabel::kUnspecified will cause an exception to
   be thrown (as @ref reserved_render_label "documented").

   @param index          The geometry index of the shape to register.
   @param shape          The shape specification to add to the render engine.
   @param properties     The perception properties provided for this geometry.
   @param X_WG           The pose of the geometry relative to the world frame W.
   @param needs_updates  If true, the geometry's pose will be updated via
                         UpdatePoses().
   @returns A unique index for the resultant render geometry (nullopt if not
            registered).
   @throws std::runtime_error if the shape is an unsupported type or if the
                              shape's RenderLabel value is
                              RenderLabel::kUnspecified or RenderLabel::kEmpty.
  */
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

  /** Renders the registered geometry into the given label image.

   @param camera                The intrinsic properties of the camera.
   @param show_window           If true, the render window will be displayed.
   @param[out] label_image_out  The rendered label image.  */
  virtual void RenderLabelImage(
      const CameraProperties& camera,
      bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const = 0;

  /** Reports the render label value this render engine has been configured to
   use.  */
  RenderLabel default_render_label() const { return default_render_label_; }

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
   engine.

   In accessing the RenderLabel property in `properties` derived class should
   _exclusively_ use GetRenderLabelOrThrow().  */
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

  /** Extracts the `(label, id)` RenderLabel property from the given
   `properties` and validates it (or the configured default if no such
   property is defined).
   @throws std::logic_error If the tested render label value is deemed invalid.
   */
  RenderLabel GetRenderLabelOrThrow(
      const PerceptionProperties& properties) const;

  /** @name   RenderLabel-Color Utilities

   Some rasterization pipelines don't support channels of
   RenderLabel::ValueType; typically, they operate in RGB color space. The
   following utilities support those pipelines by providing conversions between
   labels and colors. The mapping does _not_ produce colors that are useful
   to humans -- two labels with "near by" values will produces colors that
   most humans cannot distinguish, but the computer can. Do not use these
   utilities to produce the prototypical "colored label" images.

   The label-to-color conversion can produce one of two different color
   encodings. These encodings are not exhaustive but they are typical of the
   encodings that have proven useful. The supported color encodings consist of
   three RGB channels where each channel is either _byte-valued_ in that they
   are encoded with unsigned bytes in the range [0, 255] per channel or
   _double-valued_ such that each channel is encoded with a double in the range
   [0, 1]. Conversion to RenderLabel is only supported from byte-valued color
   values.  */
  //@{

  /** Transforms the given byte-valued RGB color value into its corresponding
   RenderLabel.  */
  static RenderLabel LabelFromColor(const systems::sensors::ColorI& color) {
    return RenderLabel(color.r | (color.g << 8), false);
  }

  /** Transforms `this` render label into a byte-valued RGB color.  */
  static systems::sensors::ColorI GetColorIFromLabel(const RenderLabel& label) {
    return systems::sensors::ColorI{label.value_ & 0xFF,
                                    (label.value_ >> 8) & 0xFF, 0};
  }

  /** Transforms `this` render label into a double-valued RGB color.  */
  static systems::sensors::ColorD GetColorDFromLabel(const RenderLabel& label) {
    systems::sensors::ColorI i_color = GetColorIFromLabel(label);
    return systems::sensors::ColorD{i_color.r / 255., i_color.g / 255.,
                                    i_color.b / 255.};
  }

  //@}

 private:
  friend class RenderEngineTester;

  // The following two maps store all registered render index values to the
  // corresponding geometry's internal index. It should be the case that the
  // keys of the two maps are disjoint and span all of the valid render index
  // values (i.e., [0, number of geometries - 1]).
  // The mapping is generally necessary to facilitate updates and geometry
  // removal.

  // The mapping from render index to internal index for those geometries whose
  // poses must be updated in UpdateVisualPose.
  std::unordered_map<RenderIndex, GeometryIndex> update_indices_;

  // The mapping from render index to internal index of all other geometries.
  std::unordered_map<RenderIndex, GeometryIndex> anchored_indices_;

  // The default render label to apply to geometries that don't otherwise
  // provide one. Default constructor is RenderLabel::kUnspecified via the
  // RenderLabel default constructor.
  RenderLabel default_render_label_{};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

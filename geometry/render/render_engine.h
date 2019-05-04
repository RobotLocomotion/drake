
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
     by an int16_t. The value stored in the channel holds a RenderLabel value
     which corresponds to an object class in the scene. Pixels attributable to
     no geometry contain the RenderLabel::kEmpty value.

     @see RenderLabel  */
class RenderEngine : public ShapeReifier {
 public:
  /** Constructs a %RenderEngine with the given default render label. If none
   is provided, the default RenderLabel will be RenderLabel::kUnspecified.
   This still allows for one more change via set_default_render_label().  */
  explicit RenderEngine(const RenderLabel& label = RenderLabel::kUnspecified)
      : default_render_label_(label) {}

  virtual ~RenderEngine() = default;

  /** Clones the render engine -- making the %RenderEngine compatible with
   copyable_unique_ptr.  */
  std::unique_ptr<RenderEngine> Clone() const;

  /** Registers a shape specification and returns the optional index of the
   corresponding render geometry. The geometry can be uniquely referenced in
   this engine (and copies of this engine) by its geometry index. The renderer
   is allowed to examine the given `properties` and choose to _not_ register
   the geometry.

   Successful registration depends on the successful validation of the
   RenderLabel value associated with the given `shape`. It looks for a
   RenderLabel in the (label, id) property; RenderLabel values in other
   properties will _not_ be validated. If no label is found, it uses the
   %RenderEngine's default value. As
   @ref reserved_render_label "documented", assigning RenderLabel::kEmpty or
   RenderLabel::kUnspecified is _not_ allowed and will cause an exception to be
   thrown. Note, the RenderLabel can end up as unspecified through a variety of
   means. For example, assigning a default-constructed RenderLabel or by
   application of the derived %RenderEngine's documented default value.

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
   @param[out] label_image_out  The rendered label image.
  */
  virtual void RenderLabelImage(
      const CameraProperties& camera,
      bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const = 0;

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

  /** @name Protocol for handling undefined RenderLabels

   @anchor setting_default_render_label
   When a geometry is registered without a RenderLabel specified (i.e., no
   (label, id) property is defined), these methods define the RenderLabel value
   that _will_ be used. The resultant behavior depends on the default value.
   For a geometry with no specified (label, id) property:

     - RenderLabel::kUnspecified or RenderLabel::kEmpty: an exception is thrown
       during RegisterVisual().
     - RenderLabel::kDoNotRender: The geometry will be allowed for RGB or depth
       images, but should _not_ be rendered in a label image.
     - Any other value: the label will simply be assigned to the geometry.

   Derived classes can set their own default RenderLabel value via the
   %RenderEngine constructor. If the derived class wishes, these methods can
   be promoted to be public to allow users the option to change their own
   default render label (with the caveat provided in
   set_default_render_label()).  */
  //@{

  /** Sets `this` render engine's default render label to `label`. See
   @ref setting_default_render_label "this documentation" for the effect of
   setting the default to various Renderlabel values. This method can be called
   only once in the lifespan of the RenderEngine; that call _must_ come before
   the first invocation of RegisterVisual().
   @throws std::logic_error if this method or RegisterVisual() has already been
                            called.  */
  void set_default_render_label(const RenderLabel& label);

  RenderLabel default_render_label() const { return default_render_label_; }

  //@}

  /** @name   RenderLabel-Color Utilities

   Some rasterization pipelines don't support channels of
   RenderLabel::ValueType; typically, they operate in RGB color space. The
   following utilities support those pipelines by providing conversions between
   labels and colors. The mapping does _not_ produce colors that are useful
   to humans -- two labels with "near by" values will produces colors that
   most humans cannot distinguish, but the computer can. Do not use these
   utilities to produce the prototypical "colored label" images.

   The label-to-color conversion can produce one of two different color
   encodings. Colors are either _byte-valued_ in that they are encoded with
   unsigned bytes in the range [0, 255] per channel or _double-valued_ such that
   each channel is encoded with a double in the range [0, 1]. Conversion to
   RenderLabel is only supported from byte-valued color values.  */
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

  /** Helper enumeration for derived classes to track if a default value can
   be configured.  */
  enum class DefaultValueState {
    /** The value is available for setting.  */
    kUnset,
    /** The value has been set and can't be set again.  */
    kSet,
    /** RegisterVisual() has been called, defaults can't change.  */
    kGeometryRegistered
  };

 private:
  friend class RenderEngineTester;

  // Validates the render label provided in the given `properties` (taking into
  // account the engine's default value). Throws if invalid, otherwise, does
  // nothing.
  void RenderLabelIsValidOrThrow(const PerceptionProperties& properties) const;

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

  // The default render label to apply to geometries that don't otherwise
  // provide one. Default constructor is RenderLabel::kUnspecified via the
  // RenderLabel default constructor.
  RenderLabel  default_render_label_{};

  // Flag that guards against the default label being set multiple times.
  DefaultValueState default_label_state_{};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

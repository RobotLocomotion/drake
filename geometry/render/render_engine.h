
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/camera_info.h"
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
class RenderEngine {
 public:
  /** Constructs a %RenderEngine with the given default render label. The
   default render label is applied to geometries that have not otherwise
   specified a (label, id) property. The value _must_ be either
   RenderLabel::kUnspecified or RenderLabel::kDontCare. (See
   @ref render_engine_default_label "this section" for more details.)

   @throws std::exception if the default render label is not one of the two
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

  virtual ~RenderEngine();

  /** Clones the render engine.

   @tparam Result must be either `std::unique_ptr<RenderEngine>` or
   `std::shared_ptr<RenderEngine>`. In C++, it defaults to unique_ptr;
   in Python, it's hard-coded to shared_ptr.

   @throws std::exception if Result is unique_ptr but this particular class only
   supports cloning for shared_ptr. */
  template <class Result = std::unique_ptr<RenderEngine>>
  Result Clone() const
    requires std::is_same_v<Result, std::unique_ptr<RenderEngine>> ||
             std::is_same_v<Result, std::shared_ptr<RenderEngine>>;

  /** @name Registering geometry with the engine

   These methods allow for requests to register new visual geometries to `this`
   %RenderEngine. The geometry is uniquely identified by the given `id`. The
   renderer is allowed to examine the given `properties` and choose to _not_
   register the geometry.

   Typically, derived classes will attempt to validate the RenderLabel value
   stored in the `(label, id)` property (or its configured default value if
   no such property exists). In that case, attempting to assign
   RenderLabel::kEmpty or RenderLabel::kUnspecified will cause an exception to
   be thrown (as @ref reserved_render_label "documented"). */
  //@{

  /** Requests registration of the given shape as a rigid geometry with this
   render engine.

   @param id             The geometry id of the shape to register.
   @param shape          The shape specification to add to the render engine.
   @param properties     The perception properties provided for this geometry.
   @param X_WG           The pose of the geometry relative to the world frame W.
   @param needs_updates  If true, the geometry's pose will be updated via
                         UpdatePoses().
   @returns True if the %RenderEngine implementation accepted the shape for
            registration.
   @throws std::exception if the shape is an unsupported type, the
                          shape's RenderLabel value is
                          RenderLabel::kUnspecified or RenderLabel::kEmpty,
                          or a geometry has already been registered with the
                          given `id`.
  */
  bool RegisterVisual(GeometryId id, const Shape& shape,
                      const PerceptionProperties& properties,
                      const math::RigidTransformd& X_WG,
                      bool needs_updates = true);

  // TODO(xuchenhan-tri): Bring RenderMesh out of internal namespace, when doing
  // that, the invariants for a RenderMesh to be valid should be verified.
  /** Requests registration of the given deformable geometry with this render
   engine.

   @experimental
   @param id             The geometry id of the shape to register.
   @param render_meshes  The mesh representations of deformable geometry in
                         its default state. A single geometry may be represented
                         by more than one render mesh. This facilitates
                         registering a geometry with more than one material for
                         rendering.
   @param properties     The perception properties provided for this geometry.
   @pre each RenderMesh in `render_meshes` is valid.
   @throws std::exception if a geometry with `id` has already been registered
           with `this` %RenderEngine.
   @throws std::exception if `render_meshes` is empty.
   @returns True if the %RenderEngine implementation accepted the geometry for
            registration. */
  bool RegisterDeformableVisual(
      GeometryId id, const std::vector<internal::RenderMesh>& render_meshes,
      const PerceptionProperties& properties);

  //@}

  /** Removes the geometry indicated by the given `id` from the engine.
   @param id    The id of the geometry to remove.
   @returns True if the geometry was removed (false implies that this id wasn't
            registered with this engine).  */
  bool RemoveGeometry(GeometryId id);

  /** Reports true if a geometry with the given `id` has been registered with
   `this` engine.  */
  bool has_geometry(GeometryId id) const;

  /** Updates the poses of all rigid geometries marked as "needing update" (see
   RegisterVisual()).

   @param X_WGs  The poses of *all* geometries in SceneGraph (measured and
                 expressed in the world frame). The pose for a geometry is
                 accessed by that geometry's id.  */
  template <typename T>
  void UpdatePoses(
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs) {
    for (const GeometryId& id : update_ids_) {
      const math::RigidTransformd X_WG =
          geometry::internal::convert_to_double(X_WGs.at(id));
      DoUpdateVisualPose(id, X_WG);
    }
  }

  // TODO(xuchenhan-tri): It may be dangerous the silently ignore the geometry
  // that is not registered. The data may be coming from the wrong source (e.g.
  // the wrong SceneGraph). But currently, render engine doesn't have a way to
  // check that.
  /** Updates the configurations of all meshes associated with the given
   deformable geometry (see RegisterDeformableVisual()). The number of elements
   in the supplied vertex position vector `q_WGs` and the vertex normal vector
   `nhats_W` must be equal to the number of render meshes registered to the
   geometry associated with `id`. Within each mesh, the vertex positions and
   normals must be ordered the same way as the vertices specified in the render
   mesh at registration when reshaped to be an Nx3 matrix with N being the
   number of vertices in the mesh.

   No-op if no geometry with the given `id` is registered with this engine.

   @experimental
   @param id       The unique identifier of a deformable geometry registered
                   with this %RenderEngine.
   @param q_WGs    The vertex positions of all meshes associated with the given
                   deformable geometry (measured and expressed in the world
                   frame).
   @param nhats_W  The vertex normals of all meshes associated with the given
                   deformable geometry (measured and expressed in the world
                   frame).
   @throws std::exception if the sizes of `q_WGs` or `nhats_W` are incompatible
           with the number of degrees of freedom of the meshes registered with
           the deformable geometry. */
  void UpdateDeformableConfigurations(
      GeometryId id, const std::vector<VectorX<double>>& q_WGs,
      const std::vector<VectorX<double>>& nhats_W);

  /** Updates the renderer's viewpoint with given pose X_WR.

   @param X_WR  The pose of renderer's viewpoint in the world coordinate
                system.  */
  virtual void UpdateViewpoint(const math::RigidTransformd& X_WR) = 0;

  /** @name Rendering using fully-specified camera models

   These methods allow for full specification of the camera model -- its
   intrinsics and render engine parameters. See the documentation of
   ColorRenderCamera and DepthRenderCamera for the full details.
   */
  //@{

  /** Renders the registered geometry into the given color (rgb) image based on
   a _fully_ specified camera.

   @param camera                The _render engine_ camera properties.
   @param[out] color_image_out  The rendered color image.
   @throws std::exception if `color_image_out` is `nullptr` or the size of the
                          given input image doesn't match the size declared in
                          `camera`.  */
  void RenderColorImage(const ColorRenderCamera& camera,
                        systems::sensors::ImageRgba8U* color_image_out) const {
    ThrowIfInvalid(camera.core().intrinsics(), color_image_out, "color");
    DoRenderColorImage(camera, color_image_out);
  }

  /** Renders the registered geometry into the given depth image based on
   a _fully_ specified camera. In contrast to the other rendering operations,
   depth images don't have an option to display the window; generally, basic
   depth images are not readily communicative to humans.

   @param camera                The _render engine_ camera properties.
   @param[out] depth_image_out  The rendered depth image.
   @throws std::exception if `depth_image_out` is `nullptr` or the size of the
                          given input image doesn't match the size declared in
                          `camera`.  */
  void RenderDepthImage(
      const DepthRenderCamera& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const {
    ThrowIfInvalid(camera.core().intrinsics(), depth_image_out, "depth");
    DoRenderDepthImage(camera, depth_image_out);
  }

  /** Renders the registered geometry into the given label image based on
   a _fully_ specified camera.

   @note This uses the ColorRenderCamera as label images are typically rendered
   to be exactly registered with a corresponding color image.

   @param camera                The _render engine_ camera properties.
   @param[out] label_image_out  The rendered label image.
   @throws std::exception if `label_image_out` is `nullptr` or the size of the
                          given input image doesn't match the size declared in
                          `camera`.  */
  void RenderLabelImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const {
    ThrowIfInvalid(camera.core().intrinsics(), label_image_out, "label");
    DoRenderLabelImage(camera, label_image_out);
  }

  //@}

  /** Reports the render label value this render engine has been configured to
   use.  */
  RenderLabel default_render_label() const { return default_render_label_; }

  /** Produces a yaml string that can be deserialized into this *particular*
   RenderEngine's type. */
  std::string GetParameterYaml() const { return DoGetParameterYaml(); }

 protected:
  // Allow derived classes to implement Cloning via copy-construction.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderEngine);

  /** The NVI-function for sub-classes to implement actual rigid geometry
   registration. If the derived class chooses not to register this particular
   shape, it should return false.

   A derived render engine can use arbitrary criteria to decide if the rigid
   geometry gets registered. In accessing the RenderLabel property in
   `properties` derived class should _exclusively_ use GetRenderLabelOrThrow().
  */
  virtual bool DoRegisterVisual(GeometryId id, const Shape& shape,
                                const PerceptionProperties& properties,
                                const math::RigidTransformd& X_WG) = 0;

  /** The NVI-function for RegisterDeformableVisual(). This function defaults to
   returning false. If the derived class chooses to register this particular
   geometry, it should return true. This function is invoked with the following
   guarantees:

      - `id` is unique (i.e. distinct from previously registered geometries).
      - `render_meshes` is non-empty.

   @experimental */
  virtual bool DoRegisterDeformableVisual(
      GeometryId id, const std::vector<internal::RenderMesh>& render_meshes,
      const PerceptionProperties& properties);

  /** The NVI-function for updating the pose of a rigid render geometry
   (identified by `id`) to the given pose X_WG.

   @param id       The id of the render geometry whose pose is being set.
   @param X_WG     The pose of the render geometry in the world frame.  */
  virtual void DoUpdateVisualPose(GeometryId id,
                                  const math::RigidTransformd& X_WG) = 0;

  /** The NVI-function for UpdateDeformableConfigurations(). It is
   invoked with the following guarantees:

    - `id` references a registered deformable geometry.
    - `q_WGs` and `nhats_W` are appropriately sized for the
       registered meshes.

   @experimental */
  virtual void DoUpdateDeformableConfigurations(
      GeometryId id, const std::vector<VectorX<double>>& q_WGs,
      const std::vector<VectorX<double>>& nhats_W);

  /** The NVI-function for removing the geometry with the given `id`.
   @param id  The id of the geometry to remove.
   @return  True if the geometry was registered with this %RenderEngine and
            removed, false if it wasn't registered in the first place.  */
  virtual bool DoRemoveGeometry(GeometryId id) = 0;

  /** The NVI-function for cloning this render engine as a shared_ptr. When not
   overridden, this base class implementation will call DoClone() to construct a
   unique_ptr clone and then promote that to a shared_ptr upon return. Note that
   in Python this is bound as simply "DoClone" not "DoCloneShared", because the
   unique_ptr flavor is nonsense in Python. */
  virtual std::shared_ptr<RenderEngine> DoCloneShared() const;

  /** The NVI-function for cloning this render engine as a unique_ptr. It must
   always be overridden, but in case a subclass does not support cloning into a
   unique_ptr, it may throw an exception. */
  virtual std::unique_ptr<RenderEngine> DoClone() const = 0;

  /** The NVI-function for rendering color with a fully-specified camera.
   When RenderColorImage calls this, it has already confirmed that
   `color_image_out` is not `nullptr` and its size is consistent with the
   camera intrinsics.

   @throws std::exception in its default implementation indicating that it has
   not been implemented. Derived %RenderEngine classes must implement this to
   support rendering color images. */
  virtual void DoRenderColorImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const;

  /** The NVI-function for rendering depth with a fully-specified camera.
   When RenderDepthImage calls this, it has already confirmed that
   `depth_image_out` is not `nullptr` and its size is consistent with the
   camera intrinsics.

   @throws std::exception in its default implementation indicating that it has
   not been implemented. Derived %RenderEngine classes must implement this to
   support rendering depth images. */
  virtual void DoRenderDepthImage(
      const DepthRenderCamera& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const;

  /** The NVI-function for rendering label with a fully-specified camera.
   When RenderLabelImage calls this, it has already confirmed that
   `label_image_out` is not `nullptr` and its size is consistent with the
   camera intrinsics.

   @throws std::exception in its default implementation indicating that it has
   not been implemented. Derived %RenderEngine classes must implement this to
   support rendering label images. */
  virtual void DoRenderLabelImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const;

  /** Extracts the `(label, id)` RenderLabel property from the given
   `properties` and validates it (or the configured default if no such
   property is defined).
   @throws std::exception If the tested render label value is deemed invalid.
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

   These utilities are provided as a _convenience_ to derived classes. Derived
   classes are not required to encode labels as colors in the same way. They are
   only obliged to return label images with proper label values according to
   the documented semantics.  */
  //@{

  /** Transforms the given RGB color into its corresponding RenderLabel.  */
  static RenderLabel MakeLabelFromRgb(uint8_t r, uint8_t g, uint8_t /* b */) {
    // The blue channel is not currently used.
    return RenderLabel(r | (g << 8), false);
  }

  /** Transforms the given render label into an RGB color.
   The alpha channel will always be 1.0. */
  static Rgba MakeRgbFromLabel(const RenderLabel& label) {
    const uint8_t r = label.value_ & 0xFF;
    const uint8_t g = (label.value_ >> 8) & 0xFF;
    return Rgba{r / 255.0, g / 255.0, /* b = */ 0.0};
  }

  //@}

  // TODO(SeanCurtis-TRI): Deprecate this API in favor of our light parameter
  // specification. First enable lights in RenderEngineVtk and confirm pass
  // through in RenderEngineGltfClient.

  /** Provides access to the light for manual configuration since it's currently
   bound to the camera position. This is a temporary measure to facilitate
   benchmarking and create visible shadows, and should not be used publicly.
   @param X_DL The pose of the light in a frame D that is attached to the camera
               position. In this frame D, the camera is located at (0, 0, 1),
               looking towards (0, 0, 0) at a distance of 1, with up being
               (0, 1, 0).  */
  virtual void SetDefaultLightPosition(const Vector3<double>& X_DL);

  template <typename ImageType>
  static void ThrowIfInvalid(const systems::sensors::CameraInfo& intrinsics,
                             const ImageType* image, const char* image_type) {
    if (image == nullptr) {
      throw std::logic_error(fmt::format(
          "Can't render a {} image. The given output image is nullptr",
          image_type));
    }
    if (image->width() != intrinsics.width() ||
        image->height() != intrinsics.height()) {
      throw std::logic_error(fmt::format(
          "The {} image to write has a size different from that specified in "
          "the camera intrinsics. Image: ({}, {}), intrinsics: ({}, {})",
          image_type, image->width(), image->height(), intrinsics.width(),
          intrinsics.height()));
    }
  }

  /** The NVI-function for GetParameterYaml(). Derived classes must implement
   this in order to support engine comparisons. */
  virtual std::string DoGetParameterYaml() const;

 private:
  friend class RenderEngineTester;

  // The following collections represent a disjoint partition of all registered
  // geometry ids (i.e., the id for a registered visual must appear in one and
  // only one of the collections).

  // The set of rigid geometry ids whose pose needs to be updated.
  // See UpdateVisualPose().
  std::unordered_set<GeometryId> update_ids_;

  // The set of geometry ids whose pose is fixed at registration time.
  std::unordered_set<GeometryId> anchored_ids_;

  // Maps ids of deformable geometries registered to this render engine to the
  // number of degrees of freedom in each of the render meshes associated with
  // this deformable geometry. Deformable geometries don't have the distinction
  // of being "dynamic" vs "anchored" as rigid geometries; they always need to
  // have their configurations updated. See UpdateDeformableConfigurations().
  std::unordered_map<GeometryId, std::vector<int>> deformable_mesh_dofs_;

  // The default render label to apply to geometries that don't otherwise
  // provide one. Default constructor is RenderLabel::kUnspecified via the
  // RenderLabel default constructor.
  RenderLabel default_render_label_{};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

#pragma once

#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/gl_renderer/buffer_dim.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"
#include "drake/geometry/render/gl_renderer/opengl_geometry.h"
#include "drake/geometry/render/gl_renderer/shader_program.h"
#include "drake/geometry/render/gl_renderer/shape_meshes.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** See documentation of MakeRenderEngineGl().  */
class RenderEngineGl final : public RenderEngine {
 public:
  /** @name Does not allow public copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor is actually protected to serve as the basis for
  // implementing the DoClone() method.
  RenderEngineGl(const RenderEngineGl&) = delete;
#endif
  RenderEngineGl& operator=(const RenderEngineGl&) = delete;
  RenderEngineGl(RenderEngineGl&&) = delete;
  RenderEngineGl& operator=(RenderEngineGl&&) = delete;
  //@}}

  RenderEngineGl();

  ~RenderEngineGl() final;

  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  /** @see RenderEngine::RenderColorImage(). Currently unimplemented. Calling
   this will throw an exception.

   Note that the display window triggered by `show_window` is shared with
   RenderLabelImage(), and only the last color or label image rendered will be
   visible in the window (once these methods are implemented).  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  /** @see RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  /** @see RenderEngine::RenderLabelImage(). Currently unimplemented. Calling
   this will throw an exception.

   Note that the display window triggered by `show_window` is shared with
   RenderColorImage(), and only the last color or label image rendered will be
   visible in the window (once these methods are implemented).  */
  void RenderLabelImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const final;

  /** @name    Shape reification  */
  //@{
  using RenderEngine::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const Box& box, void* user_data) final;
  void ImplementGeometry(const Capsule& capsule, void* user_data) final;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) final;
  void ImplementGeometry(const Mesh& mesh, void* user_data) final;
  void ImplementGeometry(const Convex& convex, void* user_data) final;
  //@}

 private:
  friend class RenderEngineGlTester;

  // Image types available. Used to index into the image-type-dependent
  // data structures.
  // TODO(SeanCurtis-TRI): Implement color image type: kColor.
  enum ImageType { kLabel = 0, kDepth, kTypeCount };

  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) final;

  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const final;

  // Copy constructor used for cloning.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Renders the object at a specific pose in the camera frame using the given
  // shader program.
  void RenderAt(const internal::ShaderProgram& shader_program,
                const Eigen::Matrix4f& X_CM, ImageType image_type) const;

  // Performs the common setup for all shape types.
  void ImplementGeometry(const internal::OpenGlGeometry& geometry,
                         void* user_data, const Vector3<double>& scale);

  // Provides triangle mesh definitions of the various canonical geometries
  // supported by this renderer: sphere, cylinder, half space, box, and mesh.
  // These update the stored OpenGlGeometry members of this class. They are
  // *not* threadsafe.
  internal::OpenGlGeometry GetSphere();
  internal::OpenGlGeometry GetCylinder();
  internal::OpenGlGeometry GetHalfSpace();
  internal::OpenGlGeometry GetBox();
  internal::OpenGlGeometry GetMesh(const std::string& filename);

  // Activates the shader for the given image type; this should be called at
  // the *top* of each Render*Image() method.
  const internal::ShaderProgram& ActivateShader(ImageType image_type) const;

  // Given the image type, returns the texture configuration for that image
  // type. These are the key arguments for glTexImage2D based on the type of
  // image. It includes:
  //   - The internal format (the number of color components)
  //   - format (the format of pixel data)
  //   - pixel type (the data type of each pixel)
  // For more details on these quantities, see
  // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml.
  static std::tuple<GLint, GLenum, GLenum> get_texture_format(
      ImageType image_type);

  // Creates a *new* render target for the given camera. This creates OpenGL
  // objects (render buffer, frame_buffer, and texture). It should only be
  // called if there is not already a cached render target for the camera's
  // reported image size (w, h) in render_targets_.
  static internal::RenderTarget CreateRenderTarget(
      const CameraProperties& camera, ImageType image_type);

  // Configures the projection matrix -- computes the matrix and sets the value
  // in the shader program's "projection_matrix" uniform. This value changes
  // on a *per-camera* basis.
  void SetGlProjectionMatrix(const internal::ShaderProgram& shader_program,
                             const CameraProperties& camera, double clip_near,
                             double clip_far) const;

  // Configures the "modelview" matrix and sets the value in the shader
  // program's "model_view_matrix" uniform. For a single camera, this changes on
  // a *per-geometry* basis.
  void SetGlModelViewMatrix(const internal::ShaderProgram& shader_program,
                            const Eigen::Matrix4f& X_CM) const;

  // Obtains the label image rendered from a specific object pose. This is
  // slower than it has to be because it does per-pixel processing on the CPU.
  // TODO(SeanCurtis-TRI): Figure out how to do all of this directly on the GPU.
  void GetLabelImage(drake::systems::sensors::ImageLabel16I* label_image_out,
                     const internal::RenderTarget& target) const;

  // Configures the OpenGL properties dependent on the camera properties. This
  // updates the cache of RenderTargets (creating one for the camera if one
  // does not already exist). As such, it is _not_ threadsafe.
  internal::RenderTarget SetCameraProperties(
      const CameraProperties& camera, const internal::ShaderProgram& program,
      double near_clip, double far_clip, ImageType image_type) const;

  // Creates an OpenGlGeometry from the mesh defined by the given `mesh_data`.
  static internal::OpenGlGeometry CreateGlGeometry(
      const internal::MeshData& mesh_data);

  // Sets the display window visibility and populates it with the _last_ image
  // rendered, if visible.
  // If `show_window` is true:
  //  - the window is made visible (or remains visible).
  //  - the window's contents display the last image rendered to `target`.
  // If `show_window` is false:
  //  - the window is made hidden (or remains hidden).
  // @pre RenderTarget's frame buffer has the same dimensions as reported by the
  // camera.
  void SetWindowVisibility(const CameraProperties& camera, bool show_window,
                           const internal::RenderTarget& target) const;

  // The cached value transformation between camera and world frame.
  math::RigidTransformd X_CW_;

  // All clones of this context share the same underlying OpenGlContext. They
  // share geometry and frame buffer objects. The following structs are either
  // shared, or copy safe w.r.t. the shared context.
  std::shared_ptr<internal::OpenGlContext> opengl_context_;

  // All of the objects below here *require* the OpenGL context. These members
  // essentially store the OpenGl "objects" (i.e., integers) that live in
  // graphics memory.

  // Shader programs for each image type.
  std::array<internal::ShaderProgram, kTypeCount> shader_programs_;

  // One OpenGlGeometry per primitive type. They represent a canonical, "unit"
  // version of the primitive type. Each instance scales and poses the
  // corresponding primitive to create arbitrarily sized geometries.
  internal::OpenGlGeometry sphere_;
  internal::OpenGlGeometry cylinder_;
  internal::OpenGlGeometry half_space_;
  internal::OpenGlGeometry box_;
  // TODO(SeanCurtis-TRI): Figure out how to re-use capsules - if two capsules
  // have the same dimensions (or are related by a *uniform* scale*), we can
  // re-use the same geometry.
  // Each capsule is unique; they cannot generally be related by a linear
  // transform (i.e., translation, rotation, and non-uniform scale).
  std::vector<internal::OpenGlGeometry> capsules_;

  // Mapping from obj filename to the mesh loaded into an OpenGlGeometry.
  std::unordered_map<std::string, internal::OpenGlGeometry> meshes_;

  // These are caches of reusable RenderTargets. There is a unique render target
  // for each unique render image size (BufferDim) and output image type. They
  // are mutable so that they can be updated in what would otherwise be a const
  // action of updating the OpenGL state for the camera.
  //
  // We need unique RenderTargets for unique output image types because the
  // type of the texture we render to differs according to output image type.
  // Thus a RenderTarget for depth cannot be used for labels (or color).
  //
  // Note: copies of this render engine share the same frame buffer objects.
  // This is *not* threadsafe!
  mutable std::array<
      std::unordered_map<internal::BufferDim, internal::RenderTarget>,
      kTypeCount>
      frame_buffers_;

  // Mapping from GeometryId to the visual data associated with that geometry.
  // When copying the render engine, this data is copied verbatim allowing the
  // copied render engine access to the same OpenGL objects in the OpenGL
  // context. However, each independent copy is allowed to independently
  // modify their copy of visuals_ (adding and removing geometries).
  std::unordered_map<GeometryId, internal::OpenGlInstance> visuals_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

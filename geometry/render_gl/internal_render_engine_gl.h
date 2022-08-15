#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gl/internal_buffer_dim.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/geometry/render_gl/internal_opengl_geometry.h"
#include "drake/geometry/render_gl/internal_shader_program.h"
#include "drake/geometry/render_gl/internal_shape_meshes.h"
#include "drake/geometry/render_gl/internal_texture_library.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

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

  /** Construct an instance of the render engine with the given `params`.  */
  explicit RenderEngineGl(RenderEngineGlParams params = {});

  ~RenderEngineGl() final;

  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  const RenderEngineGlParams& parameters() const { return parameters_; }

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

  // Mangles the mesh data before adding it to the engine to support the
  // legacy behavior of mapping mesh.obj -> mesh.png, applying it as a diffuse
  // texture, if found. When we eliminate that behavior, we can eliminate this
  // method.
  void ImplementMesh(const OpenGlGeometry& geometry, void* user_data,
                     const Vector3<double>& scale,
                     const std::string& file_name);

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

  // @see RenderEngine::DoRenderColorImage().
  void DoRenderColorImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  // @see RenderEngine::DoRenderDepthImage().
  void DoRenderDepthImage(
      const DepthRenderCamera& render_camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  // @see RenderEngine::DoRenderLabelImage().
  void DoRenderLabelImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const final;

  // Copy constructor used for cloning.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Renders all geometries which use the given shader program for the given
  // render type.
  void RenderAt(const ShaderProgram& shader_program,
                RenderType render_type) const;

  // Performs the common setup for all shape types.
  void ImplementGeometry(const OpenGlGeometry& geometry,
                         void* user_data, const Vector3<double>& scale);

  // Provides triangle mesh definitions of the various canonical geometries
  // supported by this renderer: sphere, cylinder, half space, box, and mesh.
  // These update the stored OpenGlGeometry members of this class. They are
  // *not* threadsafe.
  OpenGlGeometry GetSphere();
  OpenGlGeometry GetCylinder();
  OpenGlGeometry GetHalfSpace();
  OpenGlGeometry GetBox();
  OpenGlGeometry GetMesh(const std::string& filename);

  // Given the render type, returns the texture configuration for that render
  // type. These are the key arguments for glTexImage2D based on the render
  // type. They include:
  //   - The internal format (the number of color components)
  //   - format (the format of pixel data)
  //   - pixel type (the data type of each pixel)
  // For more details on these quantities, see
  // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml.
  static std::tuple<GLint, GLenum, GLenum> get_texture_format(
      RenderType render_type);

  // Creates a *new* render target for the given camera. This creates OpenGL
  // objects (render buffer, frame_buffer, and texture). It should only be
  // called if there is not already a cached render target for the camera's
  // reported image size (w, h) in render_targets_.
  static RenderTarget CreateRenderTarget(
      const RenderCameraCore& camera, RenderType render_type);

  // Obtains the label image rendered from a specific object pose. This is
  // slower than it has to be because it does per-pixel processing on the CPU.
  // TODO(SeanCurtis-TRI): Figure out how to do all of this directly on the GPU.
  void GetLabelImage(drake::systems::sensors::ImageLabel16I* label_image_out,
                     const RenderTarget& target) const;

  // Acquires the render target for the given camera. "Acquiring" the render
  // target guarantees that the target will be ready for receiving OpenGL
  // draw commands.
  RenderTarget GetRenderTarget(
      const RenderCameraCore& camera, RenderType render_type) const;

  // Creates an OpenGlGeometry from the mesh defined by the given `mesh_data`.
  static OpenGlGeometry CreateGlGeometry(
      const MeshData& mesh_data);

  // Sets the display window visibility and populates it with the _last_ image
  // rendered, if visible.
  // If `show_window` is true:
  //  - the window is made visible (or remains visible).
  //  - the window's contents display the last image rendered to `target`.
  // If `show_window` is false:
  //  - the window is made hidden (or remains hidden).
  // @pre RenderTarget's frame buffer has the same dimensions as reported by the
  // camera.
  void SetWindowVisibility(const RenderCameraCore& camera, bool show_window,
                           const RenderTarget& target) const;

  // Adds a shader program to the set of candidate shaders for the given render
  // type.
  ShaderId AddShader(std::unique_ptr<ShaderProgram> program,
                     RenderType render_type);

  // Given the render type and a set of perception properties, finds the
  // "most preferred" shader that supports the given properties. In this case,
  // we exploit the behind-the-scenes knowledge that Identifiers are
  // monotonically increasing. So, the shader that is added last implicitly is
  // more preferred than the earlier shader.
  ShaderProgramData GetShaderProgram(
      const PerceptionProperties& properties,
      RenderType render_type) const;

  void SetDefaultLightPosition(const Vector3<double>& light_dir_C) override {
    light_dir_C_ = light_dir_C.normalized().cast<float>();
  }

  // The cached value transformation between camera and world frames.
  math::RigidTransformd X_CW_;

  // All clones of this context share the same underlying OpenGlContext. They
  // also share the C++ abstractions of objects that *live* in the context:
  //
  //   OpenGlGeometry - the geometry buffers (copy safe)
  //   RenderTarget - frame buffer objects (copy safe)
  //   TextureLibrary - the textures (shared)
  //   ShaderProgram - the compiled shader programs (copy safe)
  //
  // So, all of these quantities are simple copy-safe POD (e.g., OpenGlGeometry)
  // or are stashed in a shared pointer.
  std::shared_ptr<OpenGlContext> opengl_context_;

  std::shared_ptr<TextureLibrary> texture_library_;

  // The engine's configuration parameters.
  RenderEngineGlParams parameters_;

  // A "shader family" is all of the shaders used to produce a particular image
  // type. Each unique shader is associated with the geometries to which it
  // applies.
  using ShaderFamily = std::map<ShaderId, std::vector<GeometryId>>;

  // Three shader families -- one for each output type.
  std::array<ShaderFamily, RenderType::kTypeCount> shader_families_;

  // The collection of all shader programs (grouped by render type).
  std::array<std::unordered_map<ShaderId,
                                copyable_unique_ptr<ShaderProgram>>,
             RenderType::kTypeCount>
      shader_programs_;

  // One OpenGlGeometry per primitive type. They represent a canonical, "unit"
  // version of the primitive type. Each instance scales and poses the
  // corresponding primitive to create arbitrarily sized geometries.
  OpenGlGeometry sphere_;
  OpenGlGeometry cylinder_;
  OpenGlGeometry half_space_;
  OpenGlGeometry box_;
  // TODO(SeanCurtis-TRI): Figure out how to re-use capsules - if two capsules
  // have the same dimensions (or are related by a *uniform* scale*), we can
  // re-use the same geometry.
  // Each capsule is unique; they cannot generally be related by a linear
  // transform (i.e., translation, rotation, and non-uniform scale).
  std::vector<OpenGlGeometry> capsules_;

  // Mapping from obj filename to the mesh loaded into an OpenGlGeometry.
  std::unordered_map<std::string, OpenGlGeometry> meshes_;

  // These are caches of reusable RenderTargets. There is a unique render target
  // for each unique image size (BufferDim) and output image type. The
  // collection is mutable so that it can be updated in what would otherwise
  // be a const action of updating the OpenGL state for the camera.
  //
  // We need unique RenderTargets for unique output image types because the
  // type of the texture we render to differs according to output image type.
  // Thus a RenderTarget for depth cannot be used for labels (or color).
  //
  // Note: copies of this render engine share the same frame buffer objects.
  // If RenderEngineGl is included in a SceneGraph and its context is cloned
  // multiple times, mutating this cache is *not* thread safe.
  mutable std::array<
      std::unordered_map<BufferDim, RenderTarget>,
      RenderType::kTypeCount>
      frame_buffers_;

  // Mapping from GeometryId to the visual data associated with that geometry.
  // When copying the render engine, this data is copied verbatim allowing the
  // copied render engine access to the same OpenGL objects in the OpenGL
  // context. However, each independent copy is allowed to independently
  // modify their copy of visuals_ (adding and removing geometries).
  std::unordered_map<GeometryId, OpenGlInstance> visuals_;

  // The direction *to* the light expressed in the camera frame.
  Vector3<float> light_dir_C_{0.0f, 0.0f, 1.0f};
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake

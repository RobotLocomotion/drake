#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gl/internal_buffer_dim.h"
#include "drake/geometry/render_gl/internal_geometry_library.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/geometry/render_gl/internal_opengl_geometry.h"
#include "drake/geometry/render_gl/internal_shader_program.h"
#include "drake/geometry/render_gl/internal_texture_library.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

/** See documentation of MakeRenderEngineGl().  */
class RenderEngineGl final : public render::RenderEngine {
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
  using render::RenderEngine::ImplementGeometry;
  void ImplementGeometry(const Box& box, void* user_data) final;
  void ImplementGeometry(const Capsule& capsule, void* user_data) final;
  void ImplementGeometry(const Convex& convex, void* user_data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) final;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const Mesh& mesh, void* user_data) final;
  void ImplementGeometry(const Sphere& sphere, void* user_data) final;
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
      const render::ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  // @see RenderEngine::DoRenderDepthImage().
  void DoRenderDepthImage(
      const render::DepthRenderCamera& render_camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  // @see RenderEngine::DoRenderLabelImage().
  void DoRenderLabelImage(
      const render::ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const final;

  // Copy constructor used for cloning.
  // Do *not* call this copy constructor directly. The resulting RenderEngineGl
  // is not complete -- it will render nothing except the background color.
  // Only call Clone() to get a copy.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Renders all geometries which use the given shader program for the given
  // render type.
  void RenderAt(const ShaderProgram& shader_program,
                RenderType render_type) const;

  // Performs the common setup for all shape types.
  void ImplementGeometry(const OpenGlGeometry& geometry,
                         void* user_data, const Vector3<double>& scale);

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
      const render::RenderCameraCore& camera, RenderType render_type);

  // Obtains the label image rendered from a specific object pose. This is
  // slower than it has to be because it does per-pixel processing on the CPU.
  // TODO(SeanCurtis-TRI): Figure out how to do all of this directly on the GPU.
  void GetLabelImage(drake::systems::sensors::ImageLabel16I* label_image_out,
                     const RenderTarget& target) const;

  // Acquires the render target for the given camera. "Acquiring" the render
  // target guarantees that the target will be ready for receiving OpenGL
  // draw commands.
  RenderTarget GetRenderTarget(const render::RenderCameraCore& camera,
                               RenderType render_type) const;

  // Sets the display window visibility and populates it with the _last_ image
  // rendered, if visible.
  // If `show_window` is true:
  //  - the window is made visible (or remains visible).
  //  - the window's contents display the last image rendered to `target`.
  // If `show_window` is false:
  //  - the window is made hidden (or remains hidden).
  // @pre RenderTarget's frame buffer has the same dimensions as reported by the
  // camera.
  void SetWindowVisibility(const render::RenderCameraCore& camera,
                           bool show_window, const RenderTarget& target) const;

  // Adds a shader program to the set of candidate shaders for the given render
  // type.
  ShaderId AddShader(std::unique_ptr<ShaderProgram> program,
                     RenderType render_type);

  // Given the render type and a set of perception properties, finds the
  // "most preferred" shader that supports the given properties. In this case,
  // we exploit the behind-the-scenes knowledge that Identifiers are
  // monotonically increasing. So, the shader that is added last implicitly is
  // more preferred than the earlier shader.
  ShaderProgramData GetShaderProgram(const PerceptionProperties& properties,
                                     RenderType render_type) const;

  void SetDefaultLightPosition(const Vector3<double>& light_dir_C) override {
    light_dir_C_ = light_dir_C.normalized().cast<float>();
  }

  // The cached value transformation between camera and world frames.
  math::RigidTransformd X_CW_;

  // The engine's configuration parameters.
  RenderEngineGlParams parameters_;

  // A simple wrapper of an `OpenGlContext` that behaves like a unique pointer
  // (in that operator*, operator-> and get() are implemented) and it confers
  // unique ownership on the underlying OpenGlContext.
  //
  // When constructed or copy constructed it automatically binds the context.
  // In constructing or cloning a RenderEngineGl instance, there is further work
  // that needs to be done beyond simply copying the C++ constructs. The
  // new RenderEngineGl instance has to properly configure the OpenGL context.
  // Automatically binding it, guarantees an available OpenGl context to all
  // subsequent operations
  //
  // This works in conjunction with the ContextUnbinder class (see below) to
  // unbind the context at the conclusion of the copy constructor, leaving the
  // cloned RenderEngine with its context unbound.
  class OpenGlContextWrapper {
   public:
    OpenGlContextWrapper() : context_(std::make_unique<OpenGlContext>()) {
      context_->MakeCurrent();
    }
    OpenGlContextWrapper(const OpenGlContextWrapper& other)
        : context_(std::make_unique<OpenGlContext>(*other.context_)) {
      context_->MakeCurrent();
    }
    OpenGlContextWrapper& operator=(const OpenGlContextWrapper&) = delete;
    OpenGlContextWrapper(OpenGlContextWrapper&&) = delete;
    OpenGlContextWrapper& operator=(OpenGlContextWrapper&&) = delete;

    const OpenGlContext& operator*() const { return *context_; }
    OpenGlContext& operator*() { return *context_; }
    const OpenGlContext* operator->() const { return context_.get(); }
    OpenGlContext* operator->() { return context_.get(); }
    const OpenGlContext* get() const { return context_.get(); }
    OpenGlContext* get() { return context_.get(); }
    // SetWindowVisibility needs to mutate the context based on camera settings.
    OpenGlContext* mutatable() const { return context_.get(); }

   private:
    std::unique_ptr<OpenGlContext> context_;
  };

  // Members *above* this member cannot depend on OpenGl. All members that
  // depend on OpenGl must follow this member. This allows the context to get
  // automatically bound prior to initializing/cloning the other members.
  OpenGlContextWrapper opengl_context_;

  // When OpenGlContext is cloned, it shares OpenGl *objects* stored on the GPU
  // with the original context. But it does not share the context *state*.
  // This class initializes the state for an OpenGL context in a consistent
  // manner and it automatically configures it during initial construction
  // and cloning.
  class ContextGlInitializer {
   public:
    ContextGlInitializer();
    ContextGlInitializer(const ContextGlInitializer&);
    ContextGlInitializer& operator=(const ContextGlInitializer&) = delete;
    ContextGlInitializer(ContextGlInitializer&&) = delete;
    ContextGlInitializer& operator=(ContextGlInitializer&&) = delete;
  };

  ContextGlInitializer initializer_;

  // All remaining data types can be simply copied by value.

  // Textures are shared across cloned OpenGl contexts. The TextureLibrary
  // contains the identifiers to those textures. A RenderEngineGl and its clones
  // all share the same TextureLibrary, so that if one instance loads a new
  // texture into the GPU, they all benefit from it (rather than blindly adding
  // the same texture redundantly).
  std::shared_ptr<TextureLibrary> texture_library_;

  GeometryLibrary geometries_;

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

  // Upon instantiation or copying, it unbinds the current OpenGlContext.
  class ContextUnbinder {
   public:
    ContextUnbinder() { OpenGlContext::ClearCurrent(); }
    ContextUnbinder(const ContextUnbinder&) : ContextUnbinder() {}
    ContextUnbinder& operator=(const ContextUnbinder&) = delete;
    ContextUnbinder(ContextUnbinder&&) = delete;
    ContextUnbinder& operator=(ContextUnbinder&&) = delete;
  };

  // This member must *always* be last. Nothing comes after it. We rely on C++
  // protocol of initializing members in declaration order to process this
  // member after all other members have completed doing any work that
  // depends on a bound OpenGlContext.
  ContextUnbinder unbinder_;
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake

#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_on_copy.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_material.h"
#include "drake/geometry/render/render_mesh.h"
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
namespace render_gl {
namespace internal {

/* See documentation of MakeRenderEngineGl().

 A discussion on using RenderEngineGl in multiple threads.
 --------------------------------------------------------

 Most importantly, a single %RenderEngineGl should *not* be exercised in
 multiple threads. One thread, one %RenderEngineGl instance.

 A %RenderEngineGl instance and its *clone* can be used in different threads
 simultaneously, but *only* the rendering APIs are threadsafe. Do not mutate the
 contents of the engine (e.g., adding/removing geometries, etc.) in parallel.

 Two independently constructed %RenderEngineGl instances can be freely used in
 different threads -- all APIs are available.

 Implementation of thread safety
 -------------------------------

 We expect the following common case:

   - A SceneGraph gets populated with some model.
   - We want to render multiple images of the contents of that world, perhaps
     from different views or even different configurations - one rendering per
     thread.

 %RenderEngineGl relies on some basic assumptions to *efficiently* achieve this:

   - The %RenderEngineGl will typically be part of a systems::Context (cloned
     from SceneGraph's model engine) and each context will be used in a single
     thread.
   - OpenGl allows for multiple OpenGl "contexts" (not the same as
     systems::Context) to share objects stored on the GPU. We use this to share
     various rendering data like textures, meshes, etc., reducing memory
     overhead and %RenderEngineGl cloning time.
   - Each RenderEngineGl has its own OpenGlContext (which corresponds to an
     "OpenGl context"). A cloned %RenderEngineGl instance gets a clone of the
     source engine's OpenGlContext. When an OpenGlContext is cloned, the clone
     shares the shareable objects of the source.

 However, in sharing data across multiple OpenGl contexts not all data is
 shareable, nor necessarily *should* be.

   - OpenGl doesn't share "container" objects. This includes the Vertex Array
     Objects (referenced in OpenGlGeometry) that stitch vertex and triangle data
     into coherent meshes.
   - OpenGl context state (such is which features are enabled/disabled, etc.)
     are unique in each context. Each OpenGlContext needs to be configured
     explicitly to have matching state.
   - If a single OpenGl shader program is used across multiple threads, they
     will conflict when setting the "uniform" parameters. For example, one
     rendering thread, in setting the color of the object it will draw, can
     change the color of an object being drawn in another thread. For now, we
     avoid this by making sure each RenderEngineGl instance has its own
     shader programs with their own uniform namespaces.

 These exceptions to the "shared" data have to be explicitly managed for each
 clone. Furthermore, the work has to be done with the engine's OpenGl context
 bound. Currently, all of this "clean up" work is done in DoClone(). */
class RenderEngineGl final : public render::RenderEngine, private ShapeReifier {
 public:
  /* @name Does not allow public copy, move, or assignment  */
  //@{

  // Note: the copy constructor is actually protected to serve as the basis for
  // implementing the DoClone() method.
  RenderEngineGl& operator=(const RenderEngineGl&) = delete;
  RenderEngineGl(RenderEngineGl&&) = delete;
  RenderEngineGl& operator=(RenderEngineGl&&) = delete;
  //@}}

  /* Constructs an instance of the render engine with the given `params`.  */
  explicit RenderEngineGl(RenderEngineGlParams params = {});

  ~RenderEngineGl() final;

  /* @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  const RenderEngineGlParams& parameters() const { return parameters_; }

  /* @name    Shape reification  */
  //@{
  using ShapeReifier::ImplementGeometry;
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

  // Initializes the OpenGl state (e.g., enabling back face culling, etc.) This
  // should be called upon construction and cloning.
  // @pre `this` engine's OpenGl context should be bound.
  void InitGlState();

  // Data to pass through the reification process.
  struct RegistrationData {
    const GeometryId id;
    const math::RigidTransformd& X_WG;
    const PerceptionProperties& properties;
    const Rgba& default_diffuse;
    bool accepted{true};
  };

  // Adds the mesh data associated with the given source to geometries_.
  // Before adding it outright, it resolves the material heuristic. If the
  // mesh data has no intrinsic material, then considers the geometry properties
  // or existence of an identically named png file.
  //
  // @pre The file key derived from the combination of `mesh_source` and
  // `is_convex` is in the meshes_ cache.
  void ImplementMeshesForSource(void* user_data, const Vector3<double>& scale,
                                const MeshSource& mesh_source, bool is_convex);

  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRegisterDeformableVisual().
  bool DoRegisterDeformableVisual(
      GeometryId id,
      const std::vector<geometry::internal::RenderMesh>& render_meshes,
      const PerceptionProperties& properties) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateDeformableConfigurations.
  void DoUpdateDeformableConfigurations(
      GeometryId id, const std::vector<VectorX<double>>& q_WGs,
      const std::vector<VectorX<double>>& nhats_W) final;

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

  // @see RenderEngine::DoGetParameterYaml().
  std::string DoGetParameterYaml() const final;

  // Copy constructor used for cloning.
  // Do *not* call this copy constructor directly. The resulting RenderEngineGl
  // is not complete -- it will render nothing except the background color.
  // Only call Clone() to get a copy.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Renders all geometries which use the given shader program for the given
  // render type.
  void RenderAt(const ShaderProgram& shader_program,
                RenderType render_type) const;

  // Creates a geometry instance from the referenced geometry data, scale, and
  // user data (e.g., GeometryId, perception properties). The instance is added
  // to visuals_.
  //
  // @param geometry_index   The index into geometries_ of the geometry used by
  //                         this instance.
  // @param user_data        The type-erased RegistrationData pointer containing
  //                         the data used during reification.
  // @param scale            The quantity for scaling the geometry *model* data
  //                         to the Drake geometry frame G. Used to construct
  //                         S_GM.
  void AddGeometryInstance(int geometry_index, void* user_data,
                           const Vector3<double>& scale);

  // The set of all geometries. These are the geometries that have the
  // identifiers for the OpenGl geometries on the GPU.
  std::vector<OpenGlGeometry> geometries_;

  // TODO(SeanCurtis-TRI): Make these threadsafe. Particularly, w.r.t. the
  // underlying OpenGl context where multiple RenderEngineGl clones share the
  // same GPU memory objects.
  // Provides triangle mesh definitions of the various canonical geometries
  // supported by this renderer: sphere, cylinder, half space, box, and mesh.
  // The returned value is an index into geometries_.
  // If the canonical mesh has not been defined yet, it will be defined.
  // These are *not* threadsafe.
  int GetSphere();
  int GetCylinder();
  int GetHalfSpace();
  int GetBox();

  // For the given convex shape, caches a render mesh based on its convex hull
  // in the geometry cache.
  //
  // What gets cached is the unscaled convex hull of the file. The scale factor
  // gets applied when the mesh is instantiated (i.e., OpenGlInstance).
  //
  // @throws if `convex.GetConvexHull()` throws (e.g., no one else has asked for
  // the hull yet, and it references an unsupported file type or the file data
  // is degenerate).
  void CacheConvexHullMesh(const Convex& convex, const RegistrationData& data);

  // For the given mesh source, parses the mesh data and attempts to place the
  // resulting render meshes in the geometry cache (i.e., a set of RenderMesh
  // instances are associated with the filename-derived key in `meshes_`).
  //
  // If the source indicates an unsupported file type, no geometry is cached,
  // and data->accepted is set to false.
  void CacheFileMeshesMaybe(const MeshSource& mesh_source,
                            RegistrationData* data);

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
  static RenderTarget CreateRenderTarget(const render::RenderCameraCore& camera,
                                         RenderType render_type);

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

  // Creates an OpenGlGeometry from the mesh defined by the given `mesh_data`.
  // The geometry is added to geometries_ and its index is returned. When
  // `is_deformable` is true, the data in the vertex buffer object of the
  // OpenGlGeometry in geometries_ indexed by the return value may be modified.
  // This function is *not* threadsafe.
  int CreateGlGeometry(const geometry::internal::RenderMesh& mesh_data,
                       bool is_deformable = false);

  // Updates the vertex arrays in all of the OpenGlGeometry instances owned by
  // this render engine.
  // @pre opengl_context_ has been bound.
  void UpdateVertexArrays();

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

  void SetDefaultLightPosition(const Vector3<double>& p_DL) override;

  const std::vector<render::LightParameter>& active_lights() const {
    if (active_lights_ == nullptr) {
      active_lights_ = parameters_.lights.size() > 0 ? &parameters_.lights
                                                     : &fallback_lights_;
    }
    return *active_lights_;
  }

  // Configures all uniforms related to lighting. This should be called once
  // upon construction, after the shaders have been instantiated. This includes
  // during cloning.
  void ConfigureLights();

  // The cached value transformation between camera and world frames.
  math::RigidTransformd X_CW_;

  // When the OpenGlContext gets copied, the copy shares the OpenGl objects
  // created in GPU memory.
  copyable_unique_ptr<OpenGlContext> opengl_context_;

  // Various C++ classes store identifiers of objects in the OpenGl context.
  // This includes:
  //
  //   OpenGlGeometry - the geometry buffers (copy safe)
  //   RenderTarget - frame buffer objects (copy safe)
  //   TextureLibrary - the textures (shared)
  //   ShaderProgram - the compiled shader programs (copy safe)
  //
  // So, all of these quantities can be safely copied verbatim.

  // OpenGl texture objects are shared across OpenGl contexts. The
  // TextureLibrary simply stores the *identifiers* to those texture objects. A
  // RenderEngineGl instance and its clones share the same TextureLibrary, so
  // that if one instance loads a new texture into the GPU, they all have
  // equal access to it (rather than blindly adding the same texture
  // redundantly).
  std::shared_ptr<TextureLibrary> texture_library_;

  // The engine's configuration parameters.
  const RenderEngineGlParams parameters_;

  // A "shader family" is all of the shaders used to produce a particular image
  // type. Each unique shader is associated with the geometries to which it
  // applies.
  using ShaderFamily = std::map<ShaderId, std::set<GeometryId>>;

  // Three shader families -- one for each output type.
  std::array<ShaderFamily, RenderType::kTypeCount> shader_families_;

  // The collection of all shader programs (grouped by render type).
  std::array<std::unordered_map<ShaderId, copyable_unique_ptr<ShaderProgram>>,
             RenderType::kTypeCount>
      shader_programs_;

  // For each primitive type, we create a single OpenGlGeometry. Each represents
  // a canonical, "unit" version of the primitive type. Each OpenGlInstance
  // scales and poses the corresponding primitive to create arbitrarily sized
  // geometries. These members are the indices into geometries_ of the primitive
  // geometry. A negative value means the primitive shape hasn't been created
  // in the OpenGlContext yet.
  //
  // There is no capsule_ because each capsule is unique. Generally, one capsule
  // cannot be realized from another capsule via a linear transformation (i.e.,
  // translation, rotation, and non-uniform scale).
  int sphere_{-1};
  int cylinder_{-1};
  int half_space_{-1};
  int box_{-1};
  // TODO(SeanCurtis-TRI): Figure out how to re-use capsules - if two capsules
  // have the same dimensions (or are related by a *uniform* scale*), we can
  // re-use the same geometry. For example, if we tracked them by "aspect ratio"
  // and allowed deviation within a small tolerance, then we could reuse them.

  // TODO(SeanCurtis-TRI): The relationships between OpenGlInstance,
  // OpenGlGeometry, RenderGlMesh, and Prop are probably overly opaque.
  // I need a succinct summary of how all of these moving pieces work together
  // with clear delineation of roles.

  // A data struct that includes the index into geometries_ containing the
  // OpenGlGeometry representation and an optional material definition of the
  // mesh. `mesh_material` will have a value *only* when the mesh provides its
  // own material definition; otherwise, it will remain std::nullopt and the
  // material will be determined during mesh instantiation.
  struct RenderGlMesh {
    // Index into geometries_ containing the instance for a RenderMesh.
    int mesh_index{};
    geometry::internal::UvState uv_state{};
    std::optional<geometry::internal::RenderMaterial> mesh_material{
        std::nullopt};
  };

  // Forward declaration of the class that processes glTF files. We declare it
  // here so that it has access to private types (e.g., RenderGlMesh).
  class GltfMeshExtractor;

  // Mapping from the obj's canonical filename to RenderGlMeshes.
  std::unordered_map<std::string, std::vector<RenderGlMesh>> meshes_;

  // This collection serves as a convenient look up when updating the vertex
  // positions and normals of deformable meshes (which get updated on a
  // per-geometry-id basis). The actual instances that get rendered are still
  // stored in visuals_, and the geometries indexed here are likewise stored in
  // geometries_. Each of the geometry indices included here should appear in
  // exactly one OpenGlInstance contained in visuals_.
  std::unordered_map<GeometryId, std::vector<int>> deformable_meshes_;

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
  mutable std::array<std::unordered_map<BufferDim, RenderTarget>,
                     RenderType::kTypeCount>
      frame_buffers_;

  // Each OpenGlInstance is associated with a single material. Some visuals
  // may be comprised of multiple instances (such as might come from an Obj or
  // glTF file). A "prop" is the collection of instances which constitute
  // one visual geometry associated with a GeometryId.
  //
  // For simple Drake primitives, the prop consists of a single instance.
  struct Prop {
    std::vector<OpenGlInstance> instances;
  };

  // Mapping from GeometryId to the visual data associated with that geometry.
  // When copying the render engine, this data is copied verbatim allowing the
  // copied render engine access to the same OpenGL objects in the OpenGL
  // context. However, each independent copy is allowed to independently
  // modify their copy of visuals_ (adding and removing geometries).
  std::unordered_map<GeometryId, Prop> visuals_;

  // Lights can be defined in the engine parameters. If no lights are defined,
  // we use the fallback_lights. Otherwise, we use the parameter lights.
  std::vector<render::LightParameter> fallback_lights_;

  // A pointer to the set of lights to actually use at runtime -- it will either
  // point to the lights in parameters_ or the fallback_lights_. Don't access
  // this directly; call active_lights() instead.
  mutable reset_on_copy<const std::vector<render::LightParameter>*>
      active_lights_{};

  // Convenience vector for scaling a geometry in x,y,z-direction by 1.0 (i.e.
  // not enlarging or shrinking the geometry) for functions that require a 3d
  // scaling.
  const Vector3<double> kUnitScale = Vector3<double>(1, 1, 1);
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake

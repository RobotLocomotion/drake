#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkActor.h>                // vtkRenderingCore
#include <vtkCommand.h>              // vtkCommonCore
#include <vtkImageExport.h>          // vtkIOImage
#include <vtkNew.h>                  // vtkCommonCore
#include <vtkPolyDataAlgorithm.h>    // vtkCommonExecutionModel
#include <vtkRenderWindow.h>         // vtkRenderingCore
#include <vtkRenderer.h>             // vtkRenderingCore
#include <vtkShaderProgram.h>        // vtkRenderingOpenGL2
#include <vtkSmartPointer.h>         // vtkCommonCore
#include <vtkTexture.h>              // vtkRenderingCore
#include <vtkWindowToImageFilter.h>  // vtkRenderingCore

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_export.h"
#include "drake/common/reset_on_copy.h"
#include "drake/common/string_unordered_map.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render/render_material.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

// A callback class for setting uniform variables used in shader programs,
// namely z_near and z_far, when vtkCommand::UpdateShaderEvent is caught.
// See also shaders::kDepthFS, this is where the variables are used.
// For the detail of VTK's callback mechanism, please refer to:
// https://www.vtk.org/doc/nightly/html/classvtkCommand.html#details
class DRAKE_NO_EXPORT ShaderCallback : public vtkCommand {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShaderCallback);

  ShaderCallback();

  static ShaderCallback* New() { return new ShaderCallback; }

  // NOLINTNEXTLINE(runtime/int): To match pre-existing APIs.
  void Execute(vtkObject*, unsigned long, void* callback_object) override {
    vtkShaderProgram* program =
        reinterpret_cast<vtkShaderProgram*>(callback_object);
    program->SetUniformf("z_near", z_near_);
    program->SetUniformf("z_far", z_far_);
  }

  void set_z_near(float z_near) { z_near_ = z_near; }

  void set_z_far(float z_far) { z_far_ = z_far; }

 private:
  float z_near_{0.f};
  float z_far_{0.f};
};

// Not for external use, RenderEngineVtk uses this to index pipelines_ member.
// Do not change, remove, or add any values.
enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

/* See documentation of MakeRenderEngineVtk().  */
class DRAKE_NO_EXPORT RenderEngineVtk : public render::RenderEngine,
                                        private ShapeReifier {
 public:
  /* @name Does not allow copy, move, or assignment  */
  //@{

  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderEngineVtk& operator=(const RenderEngineVtk&) = delete;
  RenderEngineVtk(RenderEngineVtk&&) = delete;
  RenderEngineVtk& operator=(RenderEngineVtk&&) = delete;
  //@}}

  /* Constructs the render engine from the given `parameters`.

   When one of the optional parameters is omitted, the constructed value will be
   as documented elsewhere in @ref render_engine_vtk_properties "this class".
  */
  explicit RenderEngineVtk(
      const RenderEngineVtkParams& parameters = RenderEngineVtkParams());

  ~RenderEngineVtk() override;

  /* @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) override;

  /* @name    Shape reification  */
  //@{
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Capsule& capsule, void* user_data) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  //@}

  /* @name    Access the default properties

   Provides access to the default values this instance of the render engine is
   using. These values must be set at construction.  */
  //@{

  Eigen::Vector4d default_diffuse() const { return default_diffuse_.rgba(); }

  using render::RenderEngine::default_render_label;
  //@}

 protected:
  /* Copy constructor for the purpose of cloning. */
  RenderEngineVtk(const RenderEngineVtk& other);

  /* The rendering pipeline for a single image type (color, depth, or label). */
  struct RenderingPipeline final {
    explicit RenderingPipeline(RenderEngineVtkBackend backend_in);
    ~RenderingPipeline();

    const RenderEngineVtkBackend backend;
    vtkNew<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> window;
    vtkNew<vtkWindowToImageFilter> filter;
    vtkNew<vtkImageExport> exporter;
  };

  /* Configures the VTK model to reflect the given `camera`, this includes
   render size camera intrinsics, visible windows, etc. If `show_window` is set
   to true, a named VTK window will be displayed. */
  void UpdateWindow(const render::RenderCameraCore& camera, bool show_window,
                    const RenderingPipeline& p, const char* name) const;

  /* Variant of configuring the VTK model (see previous function) that *also*
   configures the depth range. */
  void UpdateWindow(const render::DepthRenderCamera& camera,
                    const RenderingPipeline& p) const;

  /* Updates VTK rendering related objects including vtkRenderWindow,
   vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
   vtkActors' pose update for rendering. */
  static void PerformVtkUpdate(const RenderingPipeline& p);

  /* Provides access to the private data member pipelines_ by returning a
   mutable RenderingPipeline reference. Only image types in ImageType enum are
   valid. */
  RenderingPipeline& get_mutable_pipeline(ImageType image_type) const;

  /* A package of data required to register a visual geometry. This is passed as
   the void* user_data in the `ShapeReifier::ImplementGeometry()` methods. */
  struct RegistrationData {
    const PerceptionProperties& properties;
    const math::RigidTransformd& X_WG;
    const GeometryId id;
    bool accepted{true};
    std::string name;
  };

  // @see RenderEngine::DoRegisterVisual(). This throws; RenderEngineVtk wants
  // to capture names; DoRegisterNamedVisual() is the true implementation.
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRegisterNamedVisual().
  bool DoRegisterNamedVisual(GeometryId id, const Shape& shape,
                             const PerceptionProperties& properties,
                             const math::RigidTransformd& X_WG,
                             std::string_view name) override;

  // @see RenderEngine::DoRegisterDeformableVisual().
  bool DoRegisterDeformableVisual(
      GeometryId id,
      const std::vector<geometry::internal::RenderMesh>& render_meshes,
      const PerceptionProperties& properties) override;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) override;

  // @see RenderEngine::DoUpdateDeformableConfigurations().
  void DoUpdateDeformableConfigurations(
      GeometryId id, const std::vector<VectorX<double>>& q_WGs,
      const std::vector<VectorX<double>>& nhats_W) override;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) override;

  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const override;

  // @see RenderEngine::DoRenderColorImage().
  void DoRenderColorImage(
      const render::ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const override;

  // @see RenderEngine::DoRenderDepthImage().
  void DoRenderDepthImage(
      const render::DepthRenderCamera& render_camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  // @see RenderEngine::DoRenderLabelImage().
  void DoRenderLabelImage(
      const render::ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const override;

  // @see RenderEngine::DoGetParameterYaml().
  std::string DoGetParameterYaml() const override;

  // Helper function for mapping a RenderMesh instance into the appropriate VTK
  // polydata.
  void ImplementRenderMesh(geometry::internal::RenderMesh&& mesh,
                           const Vector3<double>& scale,
                           const RegistrationData& data);

  // Adds an .obj to the scene for the id currently being reified (data->id).
  // Returns true if added, false if ignored (for whatever reason).
  bool ImplementObj(const Mesh& mesh, const RegistrationData& data);

  // Adds a .gltf to the scene for the id currently being reified (data->id).
  // Returns true if added, false if ignored (for whatever reason).
  bool ImplementGltf(const Mesh& mesh, const RegistrationData& data);

 private:
  friend class RenderEngineVtkTester;

  // Our diagnostic_ object's warning callback calls this function.
  void HandleWarning(const drake::internal::DiagnosticDetail& detail) const;

  // Initializes the VTK pipelines.
  void InitializePipelines();

  // Performs the common setup for all shape types. Note, this can be called
  // multiple times for a single value of data.id. It will simply accumulate
  // multiple parts in the Prop associated with the geometry id.
  void ImplementPolyData(vtkPolyDataAlgorithm* source,
                         const geometry::internal::RenderMaterial& material,
                         const RegistrationData& data);

  void SetDefaultLightPosition(const Vector3<double>& p_DL) override;

  // Configures the render engine to require all materials to use PBR
  // interpolation. This can be mindlessly called repeatedly without harm.
  // It should be invoked any time a necessary condition is encountered to
  // ensure proper materials:
  //
  //   1) If any glTF model has been added.
  //   2) If an environment map has been added.
  //
  // Note: this affects *all* objects, whether or not a model has been
  // introduced that explicitly declares PBR materials.
  void SetPbrMaterials();

  // Setup a custom shader on the polydata mapper for rendering depth as the
  // distance from the background plane
  //
  // @pre actor is not null.
  static void SetDepthShader(vtkActor* actor);

  // Stores cached mesh data to avoid redundant re-parsing and re-instantiation
  // of geometry when the same mesh is registered multiple times.
  //
  // Each `Part` holds the VTK geometry source (a vtkPolyDataAlgorithm) and
  // the resolved material for one OBJ sub-mesh. Multiple geometry
  // registrations that reference the same OBJ file share these sources,
  // so VTK only allocates and uploads the vertex data once.
  struct CachedMesh {
    struct Part {
      // The material from the OBJ/MTL file (RenderMaterial::from_mesh_file ==
      // true), or nullopt when the file defined no material. When nullopt, the
      // per-instance material is resolved at registration time via
      // DefineMaterial() so that phong/diffuse perception properties and the
      // engine default_diffuse are both honoured correctly for each instance.
      std::optional<geometry::internal::RenderMaterial> material;
      vtkSmartPointer<vtkPolyDataAlgorithm> vtk_source;
    };
    std::vector<Part> parts;
    // The number of registered Drake geometries in _this_ engine that reference
    // this cache entry. When this count reaches zero the entry is evicted.
    int use_count{0};
  };

  // Instantiates the parts of a CachedMesh. Materials and scale factors are
  // resolved on a per-instance basis.
  void ImplementCachedMesh(const std::string& cache_key,
                           const Eigen::Vector3d& scale,
                           const RegistrationData& data);

  // A geometry is modeled with one or more "parts". A part maps to the actor
  // representing it in VTK and an optional transform mapping the actor's frame
  // A to the Drake geometry frame G. This mapping can include scaling terms.
  // If T_GA = I, then `T_GA` is set to nullptr. Otherwise, posing the actor
  // using geometry's world pose X_WG should set the transform to
  // T_WA = X_WG * T_GA.
  //
  // For the color-pipeline parts that carry a texture, `texture_key` records
  // the key into `texture_cache_` so that DoRemoveGeometry can perform
  // correct reference-count-based eviction.
  struct Part {
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkMatrix4x4> T_GA;
    // Non-null only for color-pipeline parts whose texture came from
    // texture_cache_.
    std::optional<std::string> texture_key;
  };

  // Some geometries are represented by multiple parts (such as when importing
  // a complex glTF file). A "prop" is the collection of parts which constitute
  // one visual geometry associated with a GeometryId.
  //
  // For simple Drake primitives, there will be a single "part".
  //
  // Note: this is conceptually similar to vtkAssembly. But do *not* use
  // vtkAssembly. It is incredibly buggy and causes rendering problems. See:
  // https://discourse.vtk.org/t/vtkgltfimporter-loads-textures-upside-down/12113
  // https://discourse.vtk.org/t/bug-in-vtkgltfexporter/12052
  struct Prop {
    std::vector<Part> parts;
  };

  // Three pipelines: rgb, depth, and label.
  static constexpr int kNumPipelines = 3;

  // Each geometry is represented by one "prop" per pipeline.
  using PropArray = std::array<Prop, kNumPipelines>;

  // Handles the logic for using the lights defined in the construction
  // parameters vs the built-in fallback light. The implementation should only
  // access the lights via this method (and not via the parameters nor
  // fallback_lights_).
  const std::vector<render::LightParameter>& active_lights() const {
    if (!parameters_.lights.empty()) {
      return parameters_.lights;
    }
    // fallback_lights_ may be empty if the user has specified an environment
    // map.
    return fallback_lights_;
  }

  // The engine's configuration parameters.
  const RenderEngineVtkParams parameters_;

  // VTK error and/or warning messages end up here.
  drake::internal::DiagnosticPolicy diagnostic_;

  std::array<std::unique_ptr<RenderingPipeline>, kNumPipelines> pipelines_;

  // Obnoxious bright orange.
  Rgba default_diffuse_{0.9, 0.45, 0.1, 1.0};

  // The color to clear the color buffer to.
  Rgba default_clear_color_;

  // The collection of per-geometry actors -- one actor per pipeline (color,
  // depth, and label) -- keyed by the geometry's GeometryId.
  std::unordered_map<GeometryId, PropArray> props_;

  // Cache mapping mesh source keys to parsed/rendered mesh data. The key is
  // computed from MeshSource::GetCacheKey() to uniquely identify a mesh source.
  // This eliminates redundant re-parsing and re-rendering when the same mesh
  // is registered multiple times.
  string_unordered_map<CachedMesh> mesh_cache_;

  // The texture cache entry. We track the instantiated VTK texture along with
  // our own reference count.
  struct CachedTexture {
    // The loaded texture object, shared across all geometry registrations that
    // reference the same image data and texture parameters. Erasing this
    // vtkSmartPointer removes our contribution to the texture's VTK reference
    // count, but does not guarantee immediate destruction because VTK holds
    // additional internal references (e.g. through actor bookkeeping). The
    // texture will be destroyed only once all VTK-internal references are also
    // released. The cache therefore ensures we never hold a texture alive
    // longer than necessary, without making any stronger claim about timing.
    vtkSmartPointer<vtkTexture> texture;

    // Number of color-pipeline Part instances *in this engine instance* that
    // currently hold a texture_key pointing at this entry. We cannot rely on
    // the usage counter on the vtkSmartPointer because VTK internals will
    // typically hold additional references. So, we count Drake's usages and
    // delete the cache entry when Drake's usage count reaches zero.
    int use_count{};
  };

  // Cache mapping texture keys to loaded vtkTexture objects. The key is based
  // on a hash of the image data (whether the image was specified as a path or a
  // MemoryFile). While it requires us to read the bytes from disk with every
  // reference, a cached image will still avoid decoding the image and maximize
  // reuse on the GPU.
  // The full image key contains the hash plus various suffixes to capture
  // varying per-texture parameters (OpenGl requires different texture objects
  // for different parameter combinations).
  string_unordered_map<CachedTexture> texture_cache_;

  // Maps each geometry that was registered through the mesh cache to its
  // cache key. Used by DoRemoveGeometry() to decrement use_count and evict
  // entries whose count reaches zero. Geometries not using the cache
  // (primitives, glTF, deformables) are never inserted here.
  std::unordered_map<GeometryId, std::string> geometry_mesh_keys_;

  // Lights can be defined in the engine parameters. If no lights are defined,
  // we use the fallback_lights. Otherwise, we use the parameter lights.
  // Note: We are initializing this vector with a *single* light by using the
  // LightParameter default constructor; it has been specifically designed to
  // serve as the default light.
  std::vector<render::LightParameter> fallback_lights_{};

  // If true, all newly created objects use pbr materials. As an invariant,
  // setting this to true should retroactively make all previously created
  // objects also use PBR materials (see SetPbrMaterials).
  // If false, the behavior is undefined -- the interpolation model is left to
  // VTK's default value.
  bool use_pbr_materials_{false};
};

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake

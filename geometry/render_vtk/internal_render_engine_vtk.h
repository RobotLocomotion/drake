#pragma once

#include <array>
#include <filesystem>
#include <memory>
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
#include <vtkWindowToImageFilter.h>  // vtkRenderingCore

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_export.h"
#include "drake/common/reset_on_copy.h"
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
  };

  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) override;

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

  // A geometry is modeled with one or more "parts". A part maps to the actor
  // representing it in VTK and an optional transform mapping the actor's frame
  // A to the Drake geometry frame G. This mapping can include scaling terms.
  // If T_GA = I, then `T_GA` is set to nullptr. Otherwise, posing the actor
  // using geometry's world pose X_WG should set the transform to
  // T_WA = X_WG * T_GA.
  struct Part {
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkMatrix4x4> T_GA;
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

  // By design, all of the geometry is shared across clones of the render
  // engine. This is predicated upon the idea that the geometry is *not*
  // deformable and does *not* depend on the system's pose information.
  // (If there is deformable geometry, it will have to be handled differently.)
  // Having "shared geometry" means having shared vtkPolyDataAlgorithm and
  // vtkOpenGLShaderProperty instances. The shader callback gets registered to
  // the *mapper* instances, so they all, implicitly, share the same callback.
  // Making this member static facilitates that but it does preclude the
  // possibility of simultaneous renderings with different uniform parameters.
  // Currently, this doesn't happen because drake isn't particularly thread safe
  // (or executed in such a context). However, this renderer will need some
  // formal thread safe mechanism so that it doesn't rely on that in the future.
  // TODO(SeanCurtis-TRI): This is not threadsafe; investigate mechanisms to
  // prevent undesirable behaviors if used in multi-threaded application.
  static vtkNew<ShaderCallback> uniform_setting_callback_;

  // Obnoxious bright orange.
  Rgba default_diffuse_{0.9, 0.45, 0.1, 1.0};

  // The color to clear the color buffer to.
  Rgba default_clear_color_;

  // The collection of per-geometry actors -- one actor per pipeline (color,
  // depth, and label) -- keyed by the geometry's GeometryId.
  std::unordered_map<GeometryId, PropArray> props_;

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

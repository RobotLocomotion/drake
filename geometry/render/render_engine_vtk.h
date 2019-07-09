#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkCommand.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/render/render_label.h"

#ifndef DRAKE_DOXYGEN_CXX
// This, and the ModuleInitVtkRenderingOpenGL2, provide the basis for enabling
// VTK's OpenGL2 infrastructure.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

namespace drake {
namespace geometry {
namespace render {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2(){
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

// A callback class for setting uniform variables used in shader programs,
// namely z_near and z_far, when vtkCommand::UpdateShaderEvent is caught.
// See also shaders::kDepthFS, this is where the variables are used.
// For the detail of VTK's callback mechanism, please refer to:
// https://www.vtk.org/doc/nightly/html/classvtkCommand.html#details
class ShaderCallback : public vtkCommand {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShaderCallback);

  ShaderCallback();

  static ShaderCallback* New() { return new ShaderCallback; }

  // NOLINTNEXTLINE(runtime/int): To match pre-existing APIs.
  void Execute(vtkObject*, unsigned long, void* callback_object) VTK_OVERRIDE {
    vtkShaderProgram* program =
        reinterpret_cast<vtkShaderProgram*>(callback_object);
    program->SetUniformf("z_near", z_near_);
    program->SetUniformf("z_far", z_far_);
  }

  void set_z_near(float z_near) {
    z_near_ = z_near;
  }

  void set_z_far(float z_far) {
    z_far_ = z_far;
  }

 private:
  float z_near_{0.f};
  float z_far_{0.f};
};

}  // namespace internal

#endif  // !DRAKE_DOXYGEN_CXX

/** See documentation of MakeRenderEngineVtk().  */
class RenderEngineVtk final : public RenderEngine,
                              private internal::ModuleInitVtkRenderingOpenGL2 {
 public:
  /** \name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy assignment operator is actually private to serve as the
  // basis for implementing the DoClone() method.
  RenderEngineVtk(const RenderEngineVtk&) = delete;
#endif
  RenderEngineVtk& operator=(const RenderEngineVtk&) = delete;
  RenderEngineVtk(RenderEngineVtk&&) = delete;
  RenderEngineVtk& operator=(RenderEngineVtk&&) = delete;
  //@}}

  /** Constructs the render engine from the given `parameters`.

   When one of the optional parameters is omitted, the constructed value will be
   as documented elsewhere in @ref render_engine_vtk_properties "this class".
  */
  RenderEngineVtk(
      const RenderEngineVtkParams& parameters = RenderEngineVtkParams());

  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  /** @see RenderEngine::RenderColorImage().  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  /** @see RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  /** @see RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const final;

  /** @name    Shape reification  */
  //@{
  void ImplementGeometry(const Sphere& sphere, void* user_data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const Box& box, void* user_data) final;
  void ImplementGeometry(const Mesh& mesh, void* user_data) final;
  void ImplementGeometry(const Convex& convex, void* user_data) final;
  //@}

  /** @name    Access the default properties

   Provides access to the default values this instance of the render engine is
   using. These values must be set at construction.  */
  //@{

  const Eigen::Vector4d& default_diffuse() const { return default_diffuse_; }

  using RenderEngine::default_render_label;

  //@}

 private:
  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(
      GeometryId id, const Shape& shape, const PerceptionProperties& properties,
      const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) final;

  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const final;

  // Copy constructor for the purpose of cloning.
  RenderEngineVtk(const RenderEngineVtk& other);

  // Initializes the VTK pipelines.
  void InitializePipelines();

  // Common interface for loading an obj file -- used for both mesh and convex
  // shapes.
  void ImplementObj(const std::string& file_name, double scale,
                    void* user_data);

  // Performs the common setup for all shape types.
  void ImplementGeometry(vtkPolyDataAlgorithm* source, void* user_data);

  // The rendering pipeline for a single image type (color, depth, or label).
  struct RenderingPipeline {
    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> window;
    vtkNew<vtkWindowToImageFilter> filter;
    vtkNew<vtkImageExport> exporter;
  };

  // Updates VTK rendering related objects including vtkRenderWindow,
  // vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
  // vtkActors' pose update for rendering.
  static void PerformVtkUpdate(const RenderingPipeline& p);

  // This actually modifies internal state; the pointer to a const pipeline
  // allows mutation via the contained vtkNew pointers.
  void UpdateWindow(const CameraProperties& camera, bool show_window,
                    const RenderingPipeline* p, const char* name) const;

  // Modifies the camera for the special case of the depth camera.
  void UpdateWindow(const DepthCameraProperties& camera,
                    const RenderingPipeline* p) const;

  // Three pipelines: rgb, depth, and label.
  static constexpr int kNumPipelines = 3;

  std::array<std::unique_ptr<RenderingPipeline>, kNumPipelines> pipelines_;

  // By design, all of the geometry is shared across clones of the render
  // engine. This is predicated upon the idea that the geometry is *not*
  // deformable and does *not* depend on the system's pose information.
  // (If there is deformable geometry, it will have to be handled differently.)
  // Having "shared geometry" means having shared vtkPolyDataAlgorithm and
  // vtkOpenGLPolyDataMapper instances. The shader callback gets registered to
  // the *mapper* instances, so they all, implicitly, share the same callback.
  // Making this member static facilitates that but it does preclude the
  // possibility of simultaneous renderings with different uniform parameters.
  // Currently, this doesn't happen because drake isn't particularly thread safe
  // (or executed in such a context). However, this renderer will need some
  // formal thread safe mechanism so that it doesn't rely on that in the future.
  // TODO(SeanCurtis-TRI): This is not threadsafe; investigate mechanisms to
  // prevent undesirable behaviors if used in multi-threaded application.
  static vtkNew<internal::ShaderCallback> uniform_setting_callback_;

  // Obnoxious bright orange.
  Eigen::Vector4d default_diffuse_{0.9, 0.45, 0.1, 1.0};

  // The color to clear the color buffer to.
  systems::sensors::ColorD default_clear_color_;

  // The collection of per-geometry actors (one actor per pipeline (color,
  // depth, and label) keyed by the geometry's GeometryId.
  std::unordered_map<GeometryId, std::array<vtkSmartPointer<vtkActor>, 3>>
      actors_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

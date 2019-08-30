#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkImageExport.h>
#include <vtkLight.h>
#include <vtkNew.h>
#include <vtkOSPRayPass.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_engine_ospray_factory.h"

#ifndef DRAKE_DOXYGEN_CXX
// This, and the ModuleInitVtkRenderingOpenGL2, provide the basis for enabling
// VTK's OpenGL2 infrastructure.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

namespace drake {
namespace geometry {
namespace render {
namespace internal {

struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

}  // namespace internal

/** See documentation for MakeRenderEngineOspray() for details.  */
class RenderEngineOspray final
    : public RenderEngine,
      private internal::ModuleInitVtkRenderingOpenGL2 {
 public:
  // TODO(SeanCurtis-TRI): Swap these shenanigans with a legitimate removal of
  //  all copy and move semantics. The current copy constructor's contents
  //  should simply go into the DoClone() method. The appropriate time to do
  //  this is with the VTK refactoring (resolving it here and for
  //  RenderEngineVtk). See issue #11964.
  /** @name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor is actually private to serve as the basis for
  // implementing the DoClone() method.
  RenderEngineOspray(const RenderEngineOspray&) = delete;
#endif
  RenderEngineOspray& operator=(const RenderEngineOspray&) = delete;
  RenderEngineOspray(RenderEngineOspray&&) = delete;
  RenderEngineOspray& operator=(RenderEngineOspray&&) = delete;
  //@}

  /** Constructs the render engine with the given `parameters`  */
  RenderEngineOspray(
      const RenderEngineOsprayParams& parameters = RenderEngineOsprayParams());

  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  /** @see RenderEngine::RenderColorImage().  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  /** @see RenderEngine::RenderDepthImage(). Currently throws as unimplemented.
   */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  /** @see RenderEngine::RenderLabelImage(). Currently throws as unimplemented.
   */
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

  // TODO(SeanCurtis-TRI): Figure out a "default" material - material properties
  //  and its representation (this will evolve when I support full OSPRay
  //  materials).

  const Eigen::Vector4d& default_diffuse() const { return default_diffuse_; }

  const systems::sensors::ColorD& background_color() const {
    return background_color_;
  }

  using RenderEngine::default_render_label;

  //@}

 private:
  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) final;

  // Creates a copy of the set of parameters that were supplied when this
  // instance was created.
  RenderEngineOsprayParams get_params() const;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) final;

  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const final;

  // Copy constructor for the purpose of cloning.
  RenderEngineOspray(const RenderEngineOspray& other);

  // Initializes the VTK pipelines.
  void InitializePipelines(int samples_per_pixel);

  // Common interface for loading an obj file -- used for both mesh and convex
  // shapes.
  void ImplementObj(const std::string& file_name, double scale,
                    void* user_data);

  // Performs the common setup for all shape types.
  void ImplementGeometry(vtkPolyDataAlgorithm* source, void* user_data);

  vtkNew<vtkLight> light_;

  // A single pipeline (for now): rgb.
  static constexpr int kNumPipelines = 1;

  // The rendering pipeline for a single image type.
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

  std::array<std::unique_ptr<RenderingPipeline>, kNumPipelines> pipelines_;

  vtkNew<vtkOSPRayPass> ospray_;

  // The collection of per-geometry actors (one actor per pipeline (color,
  // depth, and label) keyed by the geometry's GeometryId.
  std::unordered_map<GeometryId,
                     std::array<vtkSmartPointer<vtkActor>, kNumPipelines>>
      actors_;

  // Color to assign to objects that define no color.
  Eigen::Vector4d default_diffuse_{0.9, 0.45, 0.1, 1.0};

  // The background color -- a sky blue.
  systems::sensors::ColorD background_color_{204 / 255., 229 / 255.,
                                             255 / 255.};

  // Configuration to use path tracer or ray tracer.
  const OsprayMode render_mode_{OsprayMode::kPathTracer};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

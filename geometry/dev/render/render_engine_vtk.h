#pragma once

#include <array>
#include <memory>
#include <string>
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
#include "drake/geometry/dev/render/render_engine.h"
#include "drake/geometry/dev/render/render_label.h"
#include "drake/systems/sensors/color_palette.h"

#ifndef DRAKE_DOXYGEN_CXX
// This, and the ModuleInitVtkRenderingOpenGL2, provide the basis for enabling
// VTK's OpenGL2 infrastructure.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

namespace drake {
namespace geometry {
namespace dev {
namespace render {

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {
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

}  // namespace detail

#endif

/** Implementation of the RenderEngine using the VTK OpenGL renderer.

 @anchor render_engine_vtk_properties
 <h2>Low-fidelity geometry perception properties</h2>

 RGB images
 | Group name | Required | Property Name |  Property Type  | Property Description |
 | :--------: | :------: | :-----------: | :-------------: | :------------------- |
 |    phong   | no       | diffuse       | Eigen::Vector4d | The rgba value of the object surface |
 |    phong   | no       | diffuse_map   | std::string     | The path to a texture to apply to the geometry¹. |

 1. The semantics of the `diffuse_map` can be complex. The semantics depend on
    the specified shape:
    - Mesh: If _no_ `diffuse_map` value is specified, the renderer will look for
      a default texture. If the default texture does not exist, it applies the
      `diffuse` RGBA value. Note: specifying a _bad_ texture will skip the
      default texture and go straight to the diffuse value. The _default_
      texture is defined relative to the obj file name. If the mesh is
      `/path/to/my.obj`, its default texture would be `/path/to/my.png`.
    - All other shapes: A missing `diffuse_map` value or a bad value will both
      immediately fall through to the constant `diffuse` RGBA value.

 Depth images - no specific properties required.

 Label images
 | Group name | Required | Property Name |  Property Type  | Property Description |
 | :--------: | :------: | :-----------: | :-------------: | :------------------- |
 |   label    | no       | id            | RenderLabel     | The label to render into the image |
 If no label is provided, it uses the terrain label.
 */
class RenderEngineVtk final : public RenderEngine,
                              private detail::ModuleInitVtkRenderingOpenGL2 {
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

  RenderEngineVtk();

  /** Inherits RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const override;

  /** Inherits RenderEngine::RenderColorImage().  */
  void RenderColorImage(const CameraProperties& camera,
                        systems::sensors::ImageRgba8U* color_image_out,
                        bool show_window) const override;

  /** Inherits RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  /** Inherits RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(const CameraProperties& camera,
                        systems::sensors::ImageLabel16I* label_image_out,
                        bool show_window) const override;

  /** @name    Shape reification  */
  //@{
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;
  //@}

  /** Returns the sky's color in an RGB image. */
  const systems::sensors::ColorI& get_sky_color() const;

  /** Returns flat terrain's color in an RGB image. */
  const systems::sensors::ColorI& get_flat_terrain_color() const;

 private:
  // @see RenderEngine::DoRegisterVisual().
  optional<RenderIndex> DoRegisterVisual(
      const Shape& shape, const PerceptionProperties& properties,
      const Isometry3<double>& X_WG) override;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(const Eigen::Isometry3d& X_WG,
                          RenderIndex index) override;

  // @see RenderEngine::DoRemoveGeometry().
  optional<RenderIndex> DoRemoveGeometry(RenderIndex index) override;

  // see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const override;

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

  // TODO(SeanCurtis-TRI): Do away with this color palette; RenderLabel values
  // should be mangled into an rgb color and then the rgb color should be
  // mangled back into a 16-bit int without doing lookups. The labels can be
  // mapped to human-distinguishable colors as a post-processing operation.
  const systems::sensors::ColorPalette<RenderLabel> color_palette_;

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
  static void PerformVTKUpdate(const RenderingPipeline& p);

  // This actually modifies internal state; the pointer to a const pipeline
  // allows mutation via the contained vtkNew pointers.
  void UpdateWindow(const CameraProperties& camera, bool show_window,
                    const RenderingPipeline* p, const char* name) const;

  // Modifies the camera for the special case of the depth camera.
  void UpdateWindow(const DepthCameraProperties& camera,
                    const RenderingPipeline* p) const;

  std::array<std::unique_ptr<RenderingPipeline>, 3> pipelines_;

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
  // TODO(SeanCurtis-TRI): Add thread safety mechanisms to the renderings to
  // preclude collisions if this code is executed in a multi-thread context.
  static vtkNew<detail::ShaderCallback> uniform_setting_callback_;

  // The collection of per-geometry actors (one actor per pipeline (color,
  // depth, and label) indexed by the geometry's RenderIndex.
  std::vector<std::array<vtkSmartPointer<vtkActor>, 3>> actors_;
};

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake

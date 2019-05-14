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
#include "drake/geometry/render/render_engine.h"
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

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams  {
  /** The (optional) label to apply when none is otherwise specified.  */
  optional<RenderLabel> default_label{};

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  optional<Eigen::Vector4d> default_diffuse{};
};

/** Implementation of the RenderEngine using a VTK-based OpenGL renderer.

 @anchor render_engine_vtk_properties
 <h2>Low-fidelity geometry perception properties</h2>

 RGB images
 | Group name | Property Name | Required |  Property Type  | Property Description |
 | :--------: | :-----------: | :------: | :-------------: | :------------------- |
 |    phong   | diffuse       | no¹      | Eigen::Vector4d | The rgba² value of the object surface. |
 |    phong   | diffuse_map   | no³      | std::string     | The path to a texture to apply to the geometry. |

 ¹ If no diffuse value is given, a default rgba value will be applied. The
   default color is a bright orange. This default value can be changed to a
   different value at construction. <br>
 ² The alpha channel is currently ignored. <br>
 ³ If no path is specified, or the file cannot be read, the diffuse rgba value
   is used (or its default).

 Depth images - no specific properties required.

 Label images
 | Group name | Property Name |   Required    |  Property Type  | Property Description |
 | :--------: | :-----------: | :-----------: | :-------------: | :------------------- |
 |   label    | id            | configurable⁴ |  RenderLabel    | The label to render into the image. |

 ⁴ %RenderEngineVtk has a default render label value that is applied to any
 geometry that doesn't have a (label, id) property at registration. If a value
 is not explicitly specified, %RenderEngineVtk uses RenderLabel::kUnspecified
 as this default value. It can be explicitly set upon construction. The possible
 values for this default label and the ramifications of that choice are
 documented @ref render_engine_default_label "here".

 <h3>Geometries accepted by %RenderEngineVtk

 As documented in RenderEngine::RegisterVisual(), a RenderEngine implementation
 can use the properties found in the PerceptionProperties to determine whether
 it _accepts_ a shape provided for registration. %RenderEngineVtk makes use of
 defaults to accept _all_ geometries (assuming the properties pass validation,
 e.g., render label validation).
 <!-- TODO(SeanCurtis-TRI): Change this policy to be more selective when other
      renderers with different properties are introduced. -->
 */
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
  void UpdateViewpoint(const math::RigidTransformd& X_WR) const final;

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

  // TODO(SeanCurtis-TRI): Provide a means to set the default clear color.

  //@}

 private:
  // @see RenderEngine::DoRegisterVisual().
  optional<RenderIndex> DoRegisterVisual(
      const Shape& shape, const PerceptionProperties& properties,
      const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(RenderIndex index,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  optional<RenderIndex> DoRemoveGeometry(RenderIndex index) final;

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
  static vtkNew<internal::ShaderCallback> uniform_setting_callback_;

  // The collection of per-geometry actors (one actor per pipeline (color,
  // depth, and label) indexed by the geometry's RenderIndex.
  std::vector<std::array<vtkSmartPointer<vtkActor>, 3>> actors_;

  // Obnoxious bright orange.
  Eigen::Vector4d default_diffuse_{0.9, 0.45, 0.1, 1.0};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

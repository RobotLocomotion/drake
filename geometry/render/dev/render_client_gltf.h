#pragma once

#include <memory>
#include <string>

#include "drake/geometry/render/dev/render_client.h"
#include "drake/geometry/render/dev/render_client_gltf_factory.h"
#include "drake/geometry/render/render_engine_vtk.h"

namespace drake {
namespace geometry {
namespace render {

/** A RenderClient that exports
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> scenes to upload to a render server. */
class RenderClientGLTF : public RenderEngineVtk, public RenderClient {
 public:
  /** \name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderClientGLTF(const RenderClientGLTF&) = delete;
#endif
  RenderClientGLTF& operator=(const RenderClientGLTF&) = delete;
  RenderClientGLTF(RenderClientGLTF&&) = delete;
  RenderClientGLTF& operator=(RenderClientGLTF&&) = delete;
  virtual ~RenderClientGLTF() = default;
  //@}}

  /** Constructs the render engine from the given `parameters`.  By default the
   %RenderClientGLTF will communicate with a local server.
   \sa RenderClientGLTFParams */
  RenderClientGLTF(
      const RenderClientGLTFParams& parameters = RenderClientGLTFParams());

  // TODO(svenevs): remove this once vtkGLTFExporter is patched to invert.
  void UpdateViewpoint(const math::RigidTransformd& X_WC) override;

 protected:
  /** Copy constructor for the purpose of cloning. */
  RenderClientGLTF(const RenderClientGLTF& other);

 private:
  // \see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const override;

  /* Exports a glTF scene for the color scene, uploads it to the server,
   retrieves the rendering, and stores the final output in `color_image_out`.
   \see RenderEngineVtk::ExportColorImage. */
  void ExportColorImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const override;

  /* Exports a glTF scene for the depth scene, uploads it to the server,
   retrieves the rendering, and stores the final output in `depth_image_out`.
   \see RenderEngineVtk::ExportDepthImage. */
  void ExportDepthImage(
      const DepthRenderCamera& render_camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  /* Exports a glTF scene for the label scene, uploads it to the server,
   retrieves the rendering, and stores the final output in `label_image_out`.
   \see RenderEngineVtk::ExportLabelImage. */
  void ExportLabelImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const override;

  // TODO(svenevs): finish adding documentation.
  std::string ExportPathFor(ImageType image_type, size_t frame_id) const;

  std::string ExportScene(ImageType image_type, size_t frame_id) const;
  std::string UploadAndRender(const RenderCameraCore& core,
                              ImageType image_type,
                              const std::string& scene_path,
                              double min_depth = -1.0,
                              double max_depth = -1.0) const;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

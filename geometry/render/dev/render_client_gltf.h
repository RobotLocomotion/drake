#pragma once

#include <memory>
#include <string>

#include "drake/geometry/render/dev/render_client.h"
#include "drake/geometry/render/dev/render_client_gltf_factory.h"
#include "drake/geometry/render/render_engine_vtk.h"

namespace drake {
namespace geometry {
namespace render {

// TODO(svenevs): The RenderClientGltf class needs access to the private
// rendering pipelines of RenderEngineVtk.  We should reconsider the need for
// private friendship prior to promoting this class out of `dev`.
/** A RenderClient that exports
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> scenes to upload to a render server. */
class RenderClientGltf : public RenderEngineVtk, public RenderClient {
 public:
  /** @name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderClientGltf(const RenderClientGltf&) = delete;
#endif
  RenderClientGltf& operator=(const RenderClientGltf&) = delete;
  RenderClientGltf(RenderClientGltf&&) = delete;
  RenderClientGltf& operator=(RenderClientGltf&&) = delete;
  //@}}

  /** Constructs the render engine from the given `parameters`.  By default the
   %RenderClientGltf will communicate with a local server.
   @sa RenderClientGltfParams */
  RenderClientGltf(
      const RenderClientGltfParams& parameters = RenderClientGltfParams());

  // TODO(svenevs): remove this once vtkGLTFExporter is patched to invert.
  void UpdateViewpoint(const math::RigidTransformd& X_WC) override;

 protected:
  /** Copy constructor for the purpose of cloning. */
  RenderClientGltf(const RenderClientGltf& other);

 private:
  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const override;

  // @see RenderEngine::DoRenderColorImage().
  void DoRenderColorImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const override;

  // @see RenderEngine::DoRenderDepthImage().
  void DoRenderDepthImage(
      const DepthRenderCamera& render_camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  // @see RenderEngine::DoRenderLabelImage().
  void DoRenderLabelImage(
      const ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const override;

  /* Return the path to export a glTF scene file to for the specified
  `image_type` and `scene_id`.  The returned path is constructed as
  `{RenderClient::temp_directory()}/{scene_id}-{image_type}.gltf`. */
  std::string ExportPathFor(internal::ImageType image_type,
                            int64_t scene_id) const;

  /* Exports the `RenderEngineVtk::pipelines_[image_type]` VTK scene to a
  glTF file, returning the path to the newly exported file. */
  std::string ExportScene(internal::ImageType image_type,
                          int64_t scene_id) const;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

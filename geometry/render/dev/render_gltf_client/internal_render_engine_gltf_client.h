#pragma once

#include <memory>
#include <string>

#include "drake/geometry/render/dev/render_gltf_client/factory.h"
#include "drake/geometry/render_gltf_client/internal_render_client.h"
#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* A RenderClient that exports
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> scenes to upload to a render server. */
class RenderEngineGltfClient : public geometry::render::RenderEngineVtk {
 public:
  /* @name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderEngineGltfClient(const RenderEngineGltfClient&) = delete;
#endif
  RenderEngineGltfClient& operator=(const RenderEngineGltfClient&) = delete;
  RenderEngineGltfClient(RenderEngineGltfClient&&) = delete;
  RenderEngineGltfClient& operator=(RenderEngineGltfClient&&) = delete;
  //@}}

  /* Constructs the render engine from the given `parameters`.  By default the
   %RenderEngineGltfClient will communicate with a local server.
   @sa RenderEngineGltfClientParams */
  RenderEngineGltfClient(const RenderEngineGltfClientParams& parameters =
                             RenderEngineGltfClientParams());

  // TODO(svenevs): remove when VTK is updated, see implementation for details.
  void UpdateViewpoint(const math::RigidTransformd& X_WC) override;

 protected:
  /* Copy constructor for the purpose of cloning. */
  RenderEngineGltfClient(const RenderEngineGltfClient& other);

 private:
  // @see RenderEngine::DoClone().
  std::unique_ptr<geometry::render::RenderEngine> DoClone() const override;

  // @see RenderEngine::DoRenderColorImage().
  void DoRenderColorImage(
      const geometry::render::ColorRenderCamera& camera,
      systems::sensors::ImageRgba8U* color_image_out) const override;

  // @see RenderEngine::DoRenderDepthImage().
  void DoRenderDepthImage(
      const geometry::render::DepthRenderCamera& render_camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  // @see RenderEngine::DoRenderLabelImage().
  void DoRenderLabelImage(
      const geometry::render::ColorRenderCamera& camera,
      systems::sensors::ImageLabel16I* label_image_out) const override;

  /* Returns the path to export a glTF scene file to for the specified
  `image_type` and `scene_id`.  The returned path is constructed as
  `{RenderClient::temp_directory()}/{scene_id}-{image_type}.gltf`. */
  std::string ExportPathFor(geometry::render::internal::ImageType image_type,
                            int64_t scene_id) const;

  /* Exports the `RenderEngineVtk::pipelines_[image_type]` VTK scene to a
  glTF file, returning the path to the newly exported file. */
  std::string ExportScene(geometry::render::internal::ImageType image_type,
                          int64_t scene_id) const;

  /* Deletes the files at the paths `scene_path` and `image_path`.  Should only
   be called when `!no_cleanup()`. */
  void CleanupFrame(const std::string& scene_path,
                    const std::string& image_path) const;

  /* Helper access method for testing UpdateViewpoint matrix inversion for the
   specified image_type.  Only used for testing. */
  Eigen::Matrix4d CameraModelViewTransformMatrix(
      geometry::render::internal::ImageType image_type) const;

  friend class RenderEngineGltfClientTester;
  std::unique_ptr<RenderClient> render_client_;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

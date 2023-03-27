#pragma once

#include <memory>
#include <string>

#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gltf_client/internal_render_client.h"
#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* A RenderEngine that exports
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> scenes, uploads to a render server, and retrieves the renderings back. */
class RenderEngineGltfClient : public render_vtk::internal::RenderEngineVtk {
 public:
  /* @name Does not allow copy, move, or assignment  */
  //@{
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderEngineGltfClient& operator=(const RenderEngineGltfClient&) = delete;
  RenderEngineGltfClient(RenderEngineGltfClient&&) = delete;
  RenderEngineGltfClient& operator=(RenderEngineGltfClient&&) = delete;
  //@}}

  /* Constructs the render engine from the given `parameters`.  By default,
   RenderEngineGltfClient will communicate with a local server.
   @sa RenderEngineGltfClientParams */
  RenderEngineGltfClient(const RenderEngineGltfClientParams& parameters =
                             RenderEngineGltfClientParams());

  // TODO(svenevs): Remove when VTK is updated, see implementation for details.
  void UpdateViewpoint(const math::RigidTransformd& X_WC) override;

  const RenderEngineGltfClientParams& get_params() const {
    return render_client_->get_params();
  }

  const std::string& temp_directory() const {
    return render_client_->temp_directory();
  }

  /* (For unit testing only) Helper function to alter the behavior of
   `DoPostForm()` during unit testing. */
  void SetHttpService(std::unique_ptr<HttpService> service);

  /* (For unit testing only) Helper function for testing UpdateViewpoint matrix
   inversion for the specified image_type.
   TODO(zachfang): Remove this after VTK is updated. */
  Eigen::Matrix4d CameraModelViewTransformMatrix(
      render_vtk::internal::ImageType image_type) const;

 protected:
  /* Copy constructor for the purpose of cloning. */
  RenderEngineGltfClient(const RenderEngineGltfClient& other);

 private:
  // @see RenderEngine::DoClone().
  std::unique_ptr<render::RenderEngine> DoClone() const override;

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

  /* Exports the `RenderEngineVtk::pipelines_[image_type]` VTK scene to a
   glTF file given `export_path`. */
  void ExportScene(const std::string& export_path,
                   render_vtk::internal::ImageType image_type) const;

  std::unique_ptr<RenderClient> render_client_;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

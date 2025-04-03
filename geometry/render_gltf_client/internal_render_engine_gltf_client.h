#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "drake/common/drake_export.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gltf_client/internal_merge_gltf.h"
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
class DRAKE_NO_EXPORT RenderEngineGltfClient
    : public render_vtk::internal::RenderEngineVtk {
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

  ~RenderEngineGltfClient() override;

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
  friend class RenderEngineGltfClientTester;

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

  // @see RenderEngine::DoGetParameterYaml().
  std::string DoGetParameterYaml() const override;

  /* Exports the `RenderEngineVtk::pipelines_[image_type]` VTK scene to a
   glTF file given `export_path`. */
  void ExportScene(const std::string& export_path,
                   render_vtk::internal::ImageType image_type) const;

  /* Overrides RenderEngineVtk's default handling of the mesh types. We want to
   detect .gltf meshes and handle them in a special way. */
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) override;
  bool DoRemoveGeometry(GeometryId id) override;
  using RenderEngineVtk::ImplementGeometry;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;

  /* Adds a .gltf to the scene for the id currently being reified (data->id).
   Returns true if added, false if ignored (for whatever reason).

   Note: Even though RenderEngineVtk supports consuming and rendering glTF
   files, GltfClient handles glTF files in its own way because vtkGLTFImporter
   and vtkGLTFExporter have limited support for glTF extensions -- useful,
   common extensions. So, by injecting the files directly into the exported glTF
   file, we maintain whatever declarations the source glTF had without the lossy
   filter provided by VTK. */
  bool ImplementGltf(const Mesh& mesh,
                     const RenderEngineVtk::RegistrationData& data);

  std::unique_ptr<RenderClient> render_client_;

  struct GltfRecord {
    // The name for the .gltf file.
    //   If the glTF came from disk, it will be the file path, otherwise the
    //   filename hint associated with the in-memory mesh.
    std::string name;
    // The contents of a glTF file registered as Mesh or Convex.
    nlohmann::json contents;
    // The root nodes of the gltf file represented as a mapping from the node's
    // *local* index in the gltf to the pose of that node relative to the
    // file's frame F. Note this "pose" is not necessarily a RigidTransform. It
    // can include scale. It is the node matrix stored in the gltf.
    std::map<int, Matrix4<double>> root_nodes;
    // The anisotropic scale of the mesh.
    Vector3<double> scale = Vector3<double>::Ones();
    // The render label associated with the geometry.
    render::RenderLabel label;
  };

  // TODO(SeanCurtis-TRI) Based on the file path, I should load a gltf *one
  //  time*. Each instance of it should be re-used. Assuming that all instances
  //  of a gltf always use the *same* materials, all I would have to worry about
  //  is the transforms applied to the root nodes. That means a GltfRecord
  //  should contain a reference to the canonical mesh, and a transform instead
  //  of the `contents` it has now.
  //
  //  Exporting the scene:
  //     - Export the *mesh* once (this includes the material).
  //     - Export each instance as a (gltf node) that has the "mesh":id member
  //       (where id is the new, merged mesh id).
  //
  //  Registering logic:
  //   - Check absolute path (resolving symlinks) to file.
  //   - Load it if it isn't already loaded.
  //   - Create a GltfRecord for an instance.
  //   - Increment the counter for the content.
  //
  //  Merging logic:
  //   - For each canonical gltf with non-zero reference count
  //     - Merge everything except its root nodes.
  //     - store the mapping between canonical indices and merged indices.
  //   - For each GltfRecord
  //     - Create one node per root node.
  //       - Map to the merged mesh id.
  //       - If it has children, map canonical indices into merged indices.
  //
  //  Removing logic:
  //   - Simply remove the gltf record.
  //   - Decrement the counter with the associated json structure.
  //   - Maybe delete the canonical json.
  //
  // Note: if we allowed materials to vary between instances, we'd have to
  // distinguish at the mesh level instead of only at the node level.

  std::map<GeometryId, GltfRecord> gltfs_;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

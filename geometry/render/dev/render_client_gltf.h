#pragma once

#include <memory>
#include <string>

#include "drake/geometry/render/dev/render_client.h"
#include "drake/geometry/render/dev/render_client_gltf_factory.h"
#include "drake/geometry/render/render_engine_vtk.h"

namespace drake {
namespace geometry {
namespace render {

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
  //@}}

  virtual ~RenderClientGLTF();

  /** Constructs the render engine from the given `parameters`.
   When one of the optional parameters is omitted, the constructed value will be
   as documented elsewhere in @ref render_engine_vtk_properties "this class".
  */
  RenderClientGLTF(
      const RenderClientGLTFParams& parameters = RenderClientGLTFParams());

  using RenderEngineVtk::ImplementGeometry;

 protected:
  using RenderEngineVtk::UpdateViewpoint;
  using RenderEngineVtk::DoRegisterVisual;
  using RenderEngineVtk::DoUpdateVisualPose;
  using RenderEngineVtk::DoRemoveGeometry;

 private:
  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const override;

 protected:
  /** Copy constructor for the purpose of cloning. */
  RenderClientGLTF(const RenderClientGLTF& other);

  void ExportColorImage(systems::sensors::ImageRgba8U* buffer) const override;
  void ExportDepthImage(systems::sensors::ImageRgba8U* buffer) const override;
  void ExportLabelImage(systems::sensors::ImageRgba8U* buffer) const override;

  /** Return the file path to be used for a glTF file.
   This path is constructed from \ref temp_directory_, \ref frame_id_, and
   \p image_type.
  */
  std::string ExportPathFor(ImageType image_type, size_t frame_id) const;
  void ExportImage(ImageType image_type, size_t frame_id,
                   systems::sensors::ImageRgba8U* buffer) const;
  std::string ComputeSha256(const std::string& path) const;

  // TODO(svenevs): these methods should be restructured via RenderClient.
  virtual void PostScene(ImageType image_type, const std::string& scene_path,
                         const std::string& scene_sha256) const;
  // Returns path to where the .png was stored.
  virtual std::string GetRender(ImageType image_type,
                                const std::string& scene_path,
                                const std::string& scene_sha256) const;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

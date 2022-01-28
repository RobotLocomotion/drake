#include "drake/geometry/render/dev/render_client_gltf.h"

#include <atomic>

#include <vtkCamera.h>
#include <vtkGLTFExporter.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

#include "drake/common/filesystem.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render {

using geometry::render::RenderCameraCore;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

/* Returns the next unique-per-process integer that can uniquely identify a
 scene. */
int64_t GetNextSceneId() {
  static drake::never_destroyed<std::atomic<int64_t>> global_scene_id;
  return ++(global_scene_id.access());
}

/* Drake uses an explicit projection matrix with RenderEngineVtk to fine tune
 the displayed viewing frustum tailored to the sensor being rendered.  The
 vtkGLTFExporter, however, will be querying the renderer's vtkCamera to export
 the relevant properties.  This method modifies the related camera fields that
 are exported for glTF.  See section 3.10.3 on glTF camera projection matrices:

 https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices

 NOTE: the modifications on the vtkCamera do *NOT* need to be undone, the camera
 is modified on every frame.
 */
void SetGltfCameraPerspective(const RenderCameraCore& core, vtkCamera* camera) {
  /* Exported parameters:
   https://gitlab.kitware.com/vtk/vtk/-/blob/v9.1.0/IO/Export/vtkGLTFExporter.cxx#L352

  camValues["znear"] = cam->GetClippingRange()[0];
  camValues["zfar"] = cam->GetClippingRange()[1];
  ...
  camValues["yfov"] = vtkMath::RadiansFromDegrees(cam->GetViewAngle());
  camValues["aspectRatio"] = ren->GetTiledAspectRatio();

  For aspectRatio, note that the vtkRenderer ren's aspect ratio will be computed
  via its vtkWindow member (RenderEngineVtk::UpdateWindow already calls SetSize
  on the associated vtkWindow).

  https://gitlab.kitware.com/vtk/vtk/-/blob/v9.1.0/Rendering/Core/vtkViewport.cxx#L667
   */
  camera->ParallelProjectionOff();  // Guarantee perspective (not orthographic).
  camera->SetClippingRange(core.clipping().near(), core.clipping().far());
  // VTK wants the view angle in degrees.
  float fy = core.intrinsics().fov_y() * 180.0 / M_PI;
  camera->SetViewAngle(fy);
}

// Convert the RenderEngineVtk internal::ImageType to a RenderImageType.
// NOTE: if RenderImageType expands to have more image types, this logic will
// have to be revisited (e.g., two kinds of depth images supported).
RenderImageType InternalToRenderImageType(internal::ImageType image_type) {
  if (image_type == internal::ImageType::kColor)
    return RenderImageType::kColorRgba8U;
  else if (image_type == internal::ImageType::kDepth)
    return RenderImageType::kDepthDepth32F;
  else if (image_type == internal::ImageType::kLabel)
    return RenderImageType::kLabel16I;

  throw std::runtime_error(fmt::format(
      "RenderClientGltf: unspported internal ImageType of '{}'.", image_type));
}

std::string ImageTypeToString(internal::ImageType image_type) {
  if (image_type == internal::ImageType::kColor) {
    return "color";
  } else if (image_type == internal::ImageType::kDepth) {
    return "depth";
  } else if (image_type == internal::ImageType::kLabel) {
    return "label";
  }

  throw std::runtime_error(fmt::format(
      "RenderClientGltf: unspported internal ImageType of '{}'.", image_type));
}

void LogFrameStart(internal::ImageType image_type, int64_t scene_id) {
  drake::log()->debug("RenderClientGltf: rendering {} scene id {}.",
                      ImageTypeToString(image_type), scene_id);
}

void LogFrameGltfExportPath(internal::ImageType image_type,
                            const std::string& path) {
  drake::log()->debug("RenderClientGltf: {} scene exported to '{}'.",
                      ImageTypeToString(image_type), path);
}

void LogFrameServerResponsePath(internal::ImageType image_type,
                                const std::string& path) {
  drake::log()->debug(
      "RenderClientGltf: {} server response image saved to '{}'.",
      ImageTypeToString(image_type), path);
}

}  // namespace

RenderClientGltf::RenderClientGltf(const RenderClientGltfParams& parameters)
    : RenderEngineVtk({parameters.default_label,
                       std::nullopt,
                       // Same as RenderEngineVtkParams default clear color,
                       // though it's value is irrelevant for this renderer.
                       {204 / 255., 229 / 255., 255 / 255.}}),
      RenderClient(parameters.url, parameters.port, parameters.upload_endpoint,
                   parameters.render_endpoint, parameters.verbose,
                   parameters.no_cleanup) {}

RenderClientGltf::RenderClientGltf(const RenderClientGltf& other)
    : RenderEngineVtk(other), RenderClient(other) {}

std::unique_ptr<RenderEngine> RenderClientGltf::DoClone() const {
  return std::unique_ptr<RenderClientGltf>(new RenderClientGltf(*this));
}

void RenderClientGltf::UpdateViewpoint(const math::RigidTransformd& X_WC) {
  // TODO(svenevs): remove this once vtkGLTFExporter is patched to invert.
  // The vtkGLTFExporter sends over the camera's model view transformation, but
  // it should be sending the inverse of this matrix (bug in vtk that will be
  // patched), the specification is vague but the matrix for the camera is its
  // transformation matrix in the world, not transform the world into it.
  // NOTE: use the inverse of RigidTransformd, which will transpose the rotation
  // and invert the translation rather than doing a full matrix inverse.
  const auto drake_transform = X_WC.inverse().GetAsMatrix4();
  Eigen::Matrix4d coordinate_transform;
  // clang-format off
  coordinate_transform << 1.0,  0.0,  0.0, 0.0,
                          0.0, -1.0,  0.0, 0.0,
                          0.0,  0.0, -1.0, 0.0,
                          0.0,  0.0,  0.0, 1.0;
  // clang-format on
  Eigen::Matrix4d final_transform =
      coordinate_transform * drake_transform * coordinate_transform;
  vtkNew<vtkMatrix4x4> vtk_mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      vtk_mat->SetElement(i, j, final_transform(i, j));
    }
  }
  // Essentially the same thing as RenderEngineVtk::UpdateViewpoint.
  vtkSmartPointer<vtkTransform> vtk_transform =
      vtkSmartPointer<vtkTransform>::New();
  vtk_transform->SetMatrix(vtk_mat.GetPointer());
  for (auto& pipeline : pipelines_) {
    auto* camera = pipeline->renderer->GetActiveCamera();
    camera->SetPosition(0, 0, 0);
    camera->SetFocalPoint(0, 0, 1);
    camera->SetViewUp(0, -1, 0);
    camera->ApplyTransform(vtk_transform);
  }
}

void RenderClientGltf::DoRenderColorImage(const ColorRenderCamera& camera,
                                          ImageRgba8U* color_image_out) const {
  const auto color_scene_id = GetNextSceneId();
  if (verbose()) {
    LogFrameStart(internal::ImageType::kColor, color_scene_id);
  }

  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera.core(), camera.show_window(),
               pipelines_[internal::ImageType::kColor].get(), "Color Image");
  PerformVtkUpdate(*pipelines_[internal::ImageType::kColor]);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(
      camera.core(),
      pipelines_[internal::ImageType::kColor]->renderer->GetActiveCamera());
  const std::string scene_path =
      ExportScene(internal::ImageType::kColor, color_scene_id);
  if (verbose()) {
    LogFrameGltfExportPath(internal::ImageType::kColor, scene_path);
  }

  const std::string image_path =
      UploadAndRender(camera.core(), internal::ImageType::kColor, scene_path);
  if (verbose()) {
    LogFrameServerResponsePath(internal::ImageType::kColor, image_path);
  }

  // Load the returned image back to the drake buffer.
  LoadColorImage(image_path, color_image_out);
}

void RenderClientGltf::DoRenderDepthImage(
    const DepthRenderCamera& camera, ImageDepth32F* depth_image_out) const {
  const auto depth_scene_id = GetNextSceneId();
  if (verbose()) {
    LogFrameStart(internal::ImageType::kDepth, depth_scene_id);
  }

  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera, pipelines_[internal::ImageType::kDepth].get());
  PerformVtkUpdate(*pipelines_[internal::ImageType::kDepth]);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(
      camera.core(),
      pipelines_[internal::ImageType::kDepth]->renderer->GetActiveCamera());
  const std::string scene_path =
      ExportScene(internal::ImageType::kDepth, depth_scene_id);
  if (verbose()) {
    LogFrameGltfExportPath(internal::ImageType::kDepth, scene_path);
  }

  const double min_depth = camera.depth_range().min_depth();
  const double max_depth = camera.depth_range().max_depth();
  const std::string image_path =
      UploadAndRender(camera.core(), internal::ImageType::kDepth, scene_path,
                      min_depth, max_depth);
  if (verbose()) {
    LogFrameServerResponsePath(internal::ImageType::kDepth, image_path);
  }

  // Load the returned image back to the drake buffer.
  LoadDepthImage(image_path, depth_image_out);
}

void RenderClientGltf::DoRenderLabelImage(
    const ColorRenderCamera& camera, ImageLabel16I* label_image_out) const {
  const auto label_scene_id = GetNextSceneId();
  if (verbose()) {
    LogFrameStart(internal::ImageType::kLabel, label_scene_id);
  }

  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera.core(), camera.show_window(),
               pipelines_[internal::ImageType::kLabel].get(), "Label Image");
  PerformVtkUpdate(*pipelines_[internal::ImageType::kLabel]);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(
      camera.core(),
      pipelines_[internal::ImageType::kLabel]->renderer->GetActiveCamera());
  const std::string scene_path =
      ExportScene(internal::ImageType::kLabel, label_scene_id);
  if (verbose()) {
    LogFrameGltfExportPath(internal::ImageType::kLabel, scene_path);
  }

  const std::string image_path =
      UploadAndRender(camera.core(), internal::ImageType::kLabel, scene_path);
  if (verbose()) {
    LogFrameServerResponsePath(internal::ImageType::kLabel, image_path);
  }

  // Load the returned image back to the drake buffer.
  LoadLabelImage(image_path, label_image_out);
}

std::string RenderClientGltf::ExportPathFor(internal::ImageType image_type,
                                            int64_t scene_id) const {
  // Create e.g., {temp_directory()}/0000000000000000XYZ-color.gltf
  const drake::filesystem::path base{temp_directory()};
  // NOTE: the maximum number of digits in a int64_t is 19.
  const std::string scene{
      fmt::format("{:0>19}-{}.gltf", scene_id, ImageTypeToString(image_type))};
  return base / scene;
}

std::string RenderClientGltf::ExportScene(internal::ImageType image_type,
                                          int64_t scene_id) const {
  vtkNew<vtkGLTFExporter> gltf_exporter;
  gltf_exporter->InlineDataOn();
  gltf_exporter->SetRenderWindow(pipelines_[image_type]->window);
  const std::string scene_path = ExportPathFor(image_type, scene_id);
  gltf_exporter->SetFileName(scene_path.c_str());
  gltf_exporter->Write();
  return scene_path;
}

std::string RenderClientGltf::UploadAndRender(const RenderCameraCore& core,
                                              internal::ImageType image_type,
                                              const std::string& scene_path,
                                              double min_depth,
                                              double max_depth) const {
  if (image_type == internal::ImageType::kDepth)
    ValidDepthRangeOrThrow(min_depth, max_depth);

  const std::string scene_sha256 = ComputeSha256(scene_path);
  /* NOTE: for the mime type, the VTK glTF export produces base64 encoded data
   in a single .gltf file, this is "gltf+json" (not the binary format). */
  auto render_image_type = InternalToRenderImageType(image_type);
  UploadScene(render_image_type, scene_path, scene_sha256, "model/gltf+json");
  return RetrieveRender(core, render_image_type, scene_path, scene_sha256,
                        min_depth, max_depth);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake

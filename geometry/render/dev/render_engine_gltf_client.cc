#include "drake/geometry/render/dev/render_engine_gltf_client.h"

#include <atomic>
#include <cstdio>

#include <vtkCamera.h>
#include <vtkGLTFExporter.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkVersionMacros.h>

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

/* RenderEngineGltfClient always produces a gltf+json file.  See also:
 https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#_media_type_registrations
 */
std::string MimeType() { return "model/gltf+json"; }

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
      "RenderEngineGltfClient: unspported internal ImageType of '{}'.",
      image_type));
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
      "RenderEngineGltfClient: unspported internal ImageType of '{}'.",
      image_type));
}

void LogFrameStart(internal::ImageType image_type, int64_t scene_id) {
  drake::log()->debug("RenderEngineGltfClient: rendering {} scene id {}.",
                      ImageTypeToString(image_type), scene_id);
}

void LogFrameGltfExportPath(internal::ImageType image_type,
                            const std::string& path) {
  drake::log()->debug("RenderEngineGltfClient: {} scene exported to '{}'.",
                      ImageTypeToString(image_type), path);
}

void LogFrameServerResponsePath(internal::ImageType image_type,
                                const std::string& path) {
  drake::log()->debug(
      "RenderEngineGltfClient: {} server response image saved to '{}'.",
      ImageTypeToString(image_type), path);
}

// Delete the file and log if verbose, only call if no_cleanup() == false.
void DeleteFileAndLogIfVerbose(const std::string& path, bool verbose) {
  int failed = std::remove(path.c_str());
  if (verbose) {
    if (!failed) {
      drake::log()->debug("RenderEngineGltfClient: deleted unused file '{}'.",
                          path);
    } else {
      drake::log()->debug(
          "RenderEngineGltfClient: unable to delete file '{}' with std::remove "
          "returning code {}.",
          path, failed);
    }
  }
}

}  // namespace

RenderEngineGltfClient::RenderEngineGltfClient(
    const RenderEngineGltfClientParams& parameters)
    : RenderEngineVtk({parameters.default_label,
                       std::nullopt,
                       // Same as RenderEngineVtkParams default clear color,
                       // though it's value is irrelevant for this renderer.
                       {204 / 255., 229 / 255., 255 / 255.}}),
      RenderClient(parameters.url, parameters.port, parameters.render_endpoint,
                   parameters.verbose, parameters.no_cleanup) {}

RenderEngineGltfClient::RenderEngineGltfClient(
    const RenderEngineGltfClient& other)
    : RenderEngineVtk(other), RenderClient(other) {}

std::unique_ptr<RenderEngine> RenderEngineGltfClient::DoClone() const {
  return std::unique_ptr<RenderEngineGltfClient>(
      new RenderEngineGltfClient(*this));
}

void RenderEngineGltfClient::UpdateViewpoint(
    const math::RigidTransformd& X_WC) {
  /* The vtkGLTFExporter populates the camera matrix in "nodes" incorrectly in
   VTK 9.1.0.  It should be providing the inverted modelview transformation
   matrix since the "nodes" array is to contain global transformations.  See:

   https://gitlab.kitware.com/vtk/vtk/-/merge_requests/8883

   When VTK is updated, RenderEngineGltfClient::UpdateViewpoint, can be deleted
   as RenderEngineVtk::UpdateViewpoint will correctly update the transforms on
   the cameras. */
#if VTK_VERSION_NUMBER > VTK_VERSION_CHECK(9, 1, 0)
#error "UpdateViewpoint can be removed, modified transform no longer needed."
#endif
  /* Build the alternate transform, which consists of both an inversion of the
   input transformation as well as a coordinate system inversion.  For the
   coordinate inversion, we must account for:

   1. VTK and drake have inverted Y axis directions.
   2. The camera Z axis needs to be pointing in the opposite direction.

   RenderEngineVtk::UpdateViewpoint will result in the vtkCamera instance's
   SetPosition, SetFocalPoint, and SetViewUp methods being called, followed by
   applying the transform from drake.  See implementation of vtkCamera in VTK,
   the Set{Position,FocalPoint,ViewUp} methods result in the camera internally
   recomputing its basis -- users do not have the ability to directly control
   the modelview transform.  So we engineer a drake transform to pass back to
   RenderEngineVtk::UpdateViewpoint that will result in the final vtkCamera
   having an inverted modelview transformation than what would be needed to
   render so that the vtkGLTFExporter will export the "correct" matrix.

   When performing this coordinate-system "hand-change", we seek to invert
   both the y and z axes.  The way to do this would be to use the identity
   matrix with the axes being changed scaled by -1 (the "hand change") and
   both pre and post multiply the matrix being changed:

   | 1  0  0  0 |   | a  b  c  d |   | 1  0  0  0 |   |  a -b -c  d |
   | 0 -1  0  0 | * | e  f  g  h | * | 0 -1  0  0 | = | -e  f  g -h |
   | 0  0 -1  0 |   | i  j  k  l |   | 0  0 -1  0 |   | -i  j  k -l |
   | 0  0  0  1 |   | m  n  o  p |   | 0  0  0  1 |   |  m -n -o  p |

   which we can build directly, noting that the last row | m -n -o p | will be
   | 0 0 0 1 | (homogeneous row) and can therefore be skipped.

   NOTE: use the inverse of RigidTransformd, which will transpose the rotation
   and negate the translation rather than doing a full matrix inverse. */
  Eigen::Matrix4d hacked_matrix{X_WC.inverse().GetAsMatrix4()};
  hacked_matrix(0, 1) *= -1.0;  // -b
  hacked_matrix(0, 2) *= -1.0;  // -c
  hacked_matrix(1, 0) *= -1.0;  // -e
  hacked_matrix(1, 3) *= -1.0;  // -h
  hacked_matrix(2, 0) *= -1.0;  // -i
  hacked_matrix(2, 3) *= -1.0;  // -l
  math::RigidTransformd X_WC_hacked{hacked_matrix};
  RenderEngineVtk::UpdateViewpoint(X_WC_hacked);
}

void RenderEngineGltfClient::DoRenderColorImage(
    const ColorRenderCamera& camera, ImageRgba8U* color_image_out) const {
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

  const std::string image_path = RenderOnServer(
      camera.core(), InternalToRenderImageType(internal::ImageType::kColor),
      scene_path, MimeType());
  if (verbose()) {
    LogFrameServerResponsePath(internal::ImageType::kColor, image_path);
  }

  // Load the returned image back to the drake buffer.
  LoadColorImage(image_path, color_image_out);
  if (!no_cleanup()) CleanupFrame(scene_path, image_path);
}

void RenderEngineGltfClient::DoRenderDepthImage(
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
  const std::string image_path = RenderOnServer(
      camera.core(), InternalToRenderImageType(internal::ImageType::kDepth),
      scene_path, MimeType(), min_depth, max_depth);
  if (verbose()) {
    LogFrameServerResponsePath(internal::ImageType::kDepth, image_path);
  }

  // Load the returned image back to the drake buffer.
  LoadDepthImage(image_path, depth_image_out);
  if (!no_cleanup()) CleanupFrame(scene_path, image_path);
}

void RenderEngineGltfClient::DoRenderLabelImage(
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

  const std::string image_path = RenderOnServer(
      camera.core(), InternalToRenderImageType(internal::ImageType::kLabel),
      scene_path, MimeType());
  if (verbose()) {
    LogFrameServerResponsePath(internal::ImageType::kLabel, image_path);
  }

  // Load the returned image back to the drake buffer.
  LoadLabelImage(image_path, label_image_out);
  if (!no_cleanup()) CleanupFrame(scene_path, image_path);
}

std::string RenderEngineGltfClient::ExportPathFor(
    internal::ImageType image_type, int64_t scene_id) const {
  // Create e.g., {temp_directory()}/0000000000000000XYZ-color.gltf
  const drake::filesystem::path base{temp_directory()};
  // NOTE: the maximum number of digits in a int64_t is 19.
  const std::string scene{
      fmt::format("{:0>19}-{}.gltf", scene_id, ImageTypeToString(image_type))};
  return base / scene;
}

std::string RenderEngineGltfClient::ExportScene(internal::ImageType image_type,
                                                int64_t scene_id) const {
  vtkNew<vtkGLTFExporter> gltf_exporter;
  gltf_exporter->InlineDataOn();
  gltf_exporter->SetRenderWindow(pipelines_[image_type]->window);
  const std::string scene_path = ExportPathFor(image_type, scene_id);
  gltf_exporter->SetFileName(scene_path.c_str());
  gltf_exporter->Write();
  return scene_path;
}

void RenderEngineGltfClient::CleanupFrame(const std::string& scene_path,
                                          const std::string& image_path) const {
  DeleteFileAndLogIfVerbose(scene_path, verbose());
  DeleteFileAndLogIfVerbose(image_path, verbose());
}

}  // namespace render
}  // namespace geometry
}  // namespace drake

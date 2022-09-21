#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

#include <atomic>
#include <cstdio>
#include <string_view>
#include <utility>

#include <vtkCamera.h>
#include <vtkGLTFExporter.h>
#include <vtkMatrix4x4.h>
#include <vtkVersionMacros.h>

#include "drake/common/filesystem.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = drake::filesystem;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using geometry::render::RenderEngine;
using geometry::render::RenderEngineVtk;
using geometry::render::internal::ImageType;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

/* Returns the next unique-per-process consecutive integer that can uniquely
 identify a scene.  Drake also provides an Identifier class to produce type-safe
 program-global IDs, but it's not guaranteed to be consecutive.
 See also:
 https://github.com/RobotLocomotion/drake/blob/master/common/identifier.h
 */
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
 are exported for glTF.  As glTF has a limited definition for projection
 matrices, a Drake CameraInfo can't be fully expressed through its camera node.
 To remedy that, RenderClient::RenderOnServer() will emit the complete
 CameraInfo alongside a glTF scene file to prevent information loss.

 See "Notes on glTF Camera Specification" section on:
 https://drake.mit.edu/doxygen_cxx/group__render__engine__gltf__client__server__api.html

 See section 3.10.3 on glTF camera projection matrices:
 https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices

 NOTE: The modifications on the vtkCamera do *NOT* need to be undone, the camera
 is modified on every frame. */
void SetGltfCameraPerspective(const RenderCameraCore& core, vtkCamera* camera) {
  /* Exported parameters:
   https://gitlab.kitware.com/vtk/vtk/-/blob/v9.1.0/IO/Export/vtkGLTFExporter.cxx#L352

  camValues["znear"] = cam->GetClippingRange()[0];
  camValues["zfar"] = cam->GetClippingRange()[1];
  ...
  camValues["yfov"] = vtkMath::RadiansFromDegrees(cam->GetViewAngle());
  camValues["aspectRatio"] = ren->GetTiledAspectRatio();

  For aspectRatio, note that the vtkRenderer `ren`s aspect ratio will be
  computed via its vtkWindow member (RenderEngineVtk::UpdateWindow already calls
  SetSize on the associated vtkWindow).

  https://gitlab.kitware.com/vtk/vtk/-/blob/v9.1.0/Rendering/Core/vtkViewport.cxx#L667
   */
  camera->ParallelProjectionOff();  // Guarantee perspective (not orthographic).
  camera->SetClippingRange(core.clipping().near(), core.clipping().far());
  // VTK wants the view angle in degrees.
  const float fy = core.intrinsics().fov_y() * 180.0 / M_PI;
  camera->SetViewAngle(fy);
}

/* Converts a RenderEngineVtk ImageType to a RenderClient RenderImageType.
 NOTE: If RenderImageType expands to have more image types, e.g., two kinds of
 depth images supported, this logic will have to be revisited. */
RenderImageType VtkToRenderImageType(ImageType image_type) {
  switch (image_type) {
    case ImageType::kColor:
      return RenderImageType::kColorRgba8U;
    case ImageType::kDepth:
      return RenderImageType::kDepthDepth32F;
    case ImageType::kLabel:
      return RenderImageType::kLabel16I;
  }
  DRAKE_UNREACHABLE();
}

std::string_view ImageTypeToString(ImageType image_type) {
  switch (image_type) {
    case ImageType::kColor:
      return "color";
    case ImageType::kDepth:
      return "depth";
    case ImageType::kLabel:
      return "label";
  }
  DRAKE_UNREACHABLE();
}

void LogFrameStart(ImageType image_type, int64_t scene_id) {
  drake::log()->debug("RenderEngineGltfClient: rendering {} scene id {}.",
                      ImageTypeToString(image_type), scene_id);
}

void LogFrameGltfExportPath(ImageType image_type, const std::string& path) {
  drake::log()->debug("RenderEngineGltfClient: {} scene exported to '{}'.",
                      ImageTypeToString(image_type), path);
}

void LogFrameServerResponsePath(ImageType image_type, const std::string& path) {
  drake::log()->debug(
      "RenderEngineGltfClient: {} server response image saved to '{}'.",
      ImageTypeToString(image_type), path);
}

/* Deletes the files, i.e., `scene_path` and `image_path`, and logs if verbose.
 This function is called only when get_params().cleanup == true. */
void CleanupFrame(const std::string& scene_path, const std::string& image_path,
                  bool verbose) {
  fs::remove(scene_path);
  fs::remove(image_path);
  if (verbose) {
    drake::log()->debug(
        "RenderEngineGltfClient: deleted unused files {} and {}.", scene_path,
        image_path);
  }
}

/* Returns a formatted scene file name, e.g., 0000000000000000XYZ-color.gltf.
 NOTE: The maximum number of digits in an int64_t is 19. */
std::string GetSceneFileName(ImageType image_type, int64_t scene_id) {
  return fmt::format("{:0>19}-{}.gltf", scene_id,
                     ImageTypeToString(image_type));
}

}  // namespace

RenderEngineGltfClient::RenderEngineGltfClient(
    const RenderEngineGltfClientParams& parameters)
    : RenderEngineVtk({.default_label = parameters.default_label}),
      render_client_{std::make_unique<RenderClient>(parameters)} {}

RenderEngineGltfClient::RenderEngineGltfClient(
    const RenderEngineGltfClient& other)
    : RenderEngineVtk(other),
      render_client_(std::make_unique<RenderClient>(other.get_params())) {}

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

   When VTK is updated, RenderEngineGltfClient::UpdateViewpoint can be deleted
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

   When performing this coordinate-system "hand-change", we seek to invert both
   the y and z axes.  The way to do this would be to use the identity matrix
   with the axes being changed scaled by -1 (the "hand change") and both pre and
   post multiply the matrix being changed:

   | 1  0  0  0 |   | a  b  c  d |   | 1  0  0  0 |   |  a -b -c  d |
   | 0 -1  0  0 | * | e  f  g  h | * | 0 -1  0  0 | = | -e  f  g -h |
   | 0  0 -1  0 |   | i  j  k  l |   | 0  0 -1  0 |   | -i  j  k -l |
   | 0  0  0  1 |   | m  n  o  p |   | 0  0  0  1 |   |  m -n -o  p |

   which we can build directly, noting that the last row | m -n -o p | will be
   | 0 0 0 1 | (homogeneous row) and can therefore be skipped.

   NOTE: Use the inverse of RigidTransformd, which will transpose the rotation
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
  const int64_t color_scene_id = GetNextSceneId();
  if (get_params().verbose) {
    LogFrameStart(ImageType::kColor, color_scene_id);
  }

  const RenderingPipeline& color_pipeline =
      get_mutable_pipeline(ImageType::kColor);
  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera.core(), camera.show_window(), color_pipeline,
               "Color Image");
  PerformVtkUpdate(color_pipeline);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(camera.core(),
                           color_pipeline.renderer->GetActiveCamera());
  const std::string scene_path =
      fs::path(temp_directory()) /
      GetSceneFileName(ImageType::kColor, color_scene_id);
  ExportScene(scene_path, ImageType::kColor);
  if (get_params().verbose) {
    LogFrameGltfExportPath(ImageType::kColor, scene_path);
  }

  const std::string image_path = render_client_->RenderOnServer(
      camera.core(), VtkToRenderImageType(ImageType::kColor), scene_path,
      MimeType());
  if (get_params().verbose) {
    LogFrameServerResponsePath(ImageType::kColor, image_path);
  }

  // Load the returned image back to the drake buffer.
  render_client_->LoadColorImage(image_path, color_image_out);
  if (get_params().cleanup) {
    CleanupFrame(scene_path, image_path, get_params().verbose);
  }
}

void RenderEngineGltfClient::DoRenderDepthImage(
    const DepthRenderCamera& camera, ImageDepth32F* depth_image_out) const {
  const int64_t depth_scene_id = GetNextSceneId();

  if (get_params().verbose) {
    LogFrameStart(ImageType::kDepth, depth_scene_id);
  }

  const RenderingPipeline& depth_pipeline =
      get_mutable_pipeline(ImageType::kDepth);
  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera, depth_pipeline);
  PerformVtkUpdate(depth_pipeline);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(camera.core(),
                           depth_pipeline.renderer->GetActiveCamera());
  const std::string scene_path =
      fs::path(temp_directory()) /
      GetSceneFileName(ImageType::kDepth, depth_scene_id);
  ExportScene(scene_path, ImageType::kDepth);
  if (get_params().verbose) {
    LogFrameGltfExportPath(ImageType::kDepth, scene_path);
  }

  const std::string image_path = render_client_->RenderOnServer(
      camera.core(), VtkToRenderImageType(ImageType::kDepth), scene_path,
      MimeType(), camera.depth_range());
  if (get_params().verbose) {
    LogFrameServerResponsePath(ImageType::kDepth, image_path);
  }

  // Load the returned image back to the drake buffer.
  render_client_->LoadDepthImage(image_path, depth_image_out);
  if (get_params().cleanup) {
    CleanupFrame(scene_path, image_path, get_params().verbose);
  }
}

void RenderEngineGltfClient::DoRenderLabelImage(
    const ColorRenderCamera& camera, ImageLabel16I* label_image_out) const {
  const int64_t label_scene_id = GetNextSceneId();

  if (get_params().verbose) {
    LogFrameStart(ImageType::kLabel, label_scene_id);
  }

  const RenderingPipeline& label_pipeline =
      get_mutable_pipeline(ImageType::kLabel);
  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera.core(), camera.show_window(), label_pipeline,
               "Label Image");
  PerformVtkUpdate(label_pipeline);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(camera.core(),
                           label_pipeline.renderer->GetActiveCamera());
  const std::string scene_path =
      fs::path(temp_directory()) /
      GetSceneFileName(ImageType::kLabel, label_scene_id);
  ExportScene(scene_path, ImageType::kLabel);
  if (get_params().verbose) {
    LogFrameGltfExportPath(ImageType::kLabel, scene_path);
  }

  const std::string image_path = render_client_->RenderOnServer(
      camera.core(), VtkToRenderImageType(ImageType::kLabel), scene_path,
      MimeType());
  if (get_params().verbose) {
    LogFrameServerResponsePath(ImageType::kLabel, image_path);
  }

  // Load the returned image back to the drake buffer.
  render_client_->LoadLabelImage(image_path, label_image_out);
  if (get_params().cleanup) {
    CleanupFrame(scene_path, image_path, get_params().verbose);
  }
}

void RenderEngineGltfClient::ExportScene(const std::string& export_path,
                                         ImageType image_type) const {
  vtkNew<vtkGLTFExporter> gltf_exporter;
  gltf_exporter->InlineDataOn();
  gltf_exporter->SetRenderWindow(get_mutable_pipeline(image_type).window);
  gltf_exporter->SetFileName(export_path.c_str());
  gltf_exporter->Write();
}

Eigen::Matrix4d RenderEngineGltfClient::CameraModelViewTransformMatrix(
    ImageType image_type) const {
  vtkCamera* cam = get_mutable_pipeline(image_type).renderer->GetActiveCamera();
  const vtkMatrix4x4* vtk_mat = cam->GetModelViewTransformMatrix();
  Eigen::Matrix4d ret;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ret(i, j) = vtk_mat->GetElement(i, j);
    }
  }
  return ret;
}

void RenderEngineGltfClient::SetHttpService(
    std::unique_ptr<HttpService> service) {
  render_client_->SetHttpService(std::move(service));
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

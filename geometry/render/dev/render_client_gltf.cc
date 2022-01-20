#include "drake/geometry/render/dev/render_client_gltf.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <vtkCamera.h>
#include <vtkGLTFExporter.h>
#include <vtkImageExport.h>
#include <vtkIndent.h>
#include <vtkMatrix4x4.h>
#include <vtkPNGReader.h>

#include "drake/common/filesystem.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/render/render_engine_vtk_base.h"
#include "drake/geometry/render/shaders/depth_shaders.h"
#include "drake/geometry/render/vtk_util.h"
#include "drake/systems/sensors/color_palette.h"

namespace drake {
namespace geometry {
namespace render {

using systems::sensors::CameraInfo;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;

namespace {

/* Drake uses an explicit projection matrix with RenderEngineVtk to fine tune
 the displayed viewing frustum tailored to the sensor being rendered.  The
 vtkGLTFExporter, however, will be querying the renderer's vtkCamera to export
 the relevant properties.  This method modifies the related camera fields that
 are exported for glTF.  See section 3.10.3 on glTF camera projection matrices:

 https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices

 NOTE: the modifications on the vtkCamera do *NOT* need to be undone, the camera
 is modified on every frame.
 */
void SetGltfCameraPerspective(vtkCamera* camera, const RenderCameraCore& core) {
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

}  // namespace

using drake::geometry::render::vtk_util::ConvertToVtkTransform;
using Eigen::Vector3d;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

RenderClientGLTF::RenderClientGLTF(const RenderClientGLTFParams& parameters)
    : RenderEngineVtk({parameters.default_label,
                       std::nullopt,
                       // Same as RenderEngineVtkParams default clear color,
                       // though it's value is irrelevant for this renderer.
                       {204 / 255., 229 / 255., 255 / 255.}}),
      RenderClient(parameters.url, parameters.port, parameters.upload_endpoint,
                   parameters.render_endpoint, parameters.verbose,
                   parameters.no_cleanup) {}

RenderClientGLTF::RenderClientGLTF(const RenderClientGLTF& other)
    : RenderEngineVtk(other), RenderClient(other) {}

std::unique_ptr<RenderEngine> RenderClientGLTF::DoClone() const {
  return std::unique_ptr<RenderClientGLTF>(new RenderClientGLTF(*this));
}

void RenderClientGLTF::UpdateViewpoint(const math::RigidTransformd& X_WC) {
  // TODO(svenevs): remove this once vtkGLTFExporter is patched to invert.
  // The vtkGLTFExporter sends over the camera's model view transformation, but
  // it should be sending the inverse of this matrix (bug in vtk that will be
  // patched), the specification is vague but the matrix for the camera is its
  // transformation matrix in the world, not transform the world into it.
  // NOTE: use the inverse of RigidTransformd, which will transpose the rotation
  // and invert the translation rather than doing a full matrix inverse.
  auto lh_xform = X_WC.inverse().GetAsMatrix4();
  Eigen::Matrix4d hand_change;
  // clang-format off
  hand_change << 1.0,  0.0,  0.0, 0.0,
                 0.0, -1.0,  0.0, 0.0,
                 0.0,  0.0, -1.0, 0.0,
                 0.0,  0.0,  0.0, 1.0;
  // clang-format on
  Eigen::Matrix4d xform = hand_change * lh_xform * hand_change;
  vtkNew<vtkMatrix4x4> vtk_mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      vtk_mat->SetElement(i, j, xform(i, j));
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

void RenderClientGLTF::ExportColorImage(const ColorRenderCamera& camera,
                                        ImageRgba8U* color_image_out) const {
  // TODO(svenevs): good thread-safe location for tracking color frame id?
  static size_t color_frame_id{0};

  // TODO(svenevs): is logging desired?  Use drake's log()?
  if (verbose()) {
    std::cout << "RenderClientGLTF color frame: " << color_frame_id << '\n';
  }

  // Export and render the glTF scene.
  SetGltfCameraPerspective(
      pipelines_[ImageType::kColor]->renderer->GetActiveCamera(),
      camera.core());
  const std::string scene_path = ExportScene(ImageType::kColor, color_frame_id);
  if (verbose()) {
    std::cout << "  Scene exported to: " << scene_path << '\n';
  }

  const std::string img_path =
      UploadAndRender(camera.core(), ImageType::kColor, scene_path);
  if (verbose()) {
    std::cout << "  Server response image: " << img_path << '\n';
  }

  // Load the returned image back to the drake buffer.
  LoadColorImage(img_path, color_image_out);
  ++color_frame_id;
}

void RenderClientGLTF::ExportDepthImage(const DepthRenderCamera& camera,
                                        ImageDepth32F* depth_image_out) const {
  // TODO(svenevs): good thread-safe location for tracking depth frame id?
  static size_t depth_frame_id{0};

  // TODO(svenevs): is logging desired?  Use drake's log()?
  if (verbose()) {
    std::cout << "RenderClientGLTF depth frame: " << depth_frame_id << '\n';
  }

  // Export and render the glTF scene.
  SetGltfCameraPerspective(
      pipelines_[ImageType::kDepth]->renderer->GetActiveCamera(),
      camera.core());
  const std::string scene_path = ExportScene(ImageType::kDepth, depth_frame_id);
  if (verbose()) {
    std::cout << "  Scene exported to: " << scene_path << '\n';
  }

  const double min_depth = camera.depth_range().min_depth();
  const double max_depth = camera.depth_range().max_depth();
  const std::string img_path = UploadAndRender(
      camera.core(), ImageType::kDepth, scene_path, min_depth, max_depth);
  if (verbose()) {
    std::cout << "  Server response image: " << img_path << '\n';
  }

  // Load the returned image back to the drake buffer.
  LoadDepthImage(img_path, depth_image_out);
  ++depth_frame_id;
}

void RenderClientGLTF::ExportLabelImage(const ColorRenderCamera& camera,
                                        ImageLabel16I* label_image_out) const {
  // TODO(svenevs): good thread-safe location for tracking label frame id?
  static size_t label_frame_id{0};

  // TODO(svenevs): is logging desired?  Use drake's log()?
  if (verbose()) {
    std::cout << "RenderClientGLTF label frame: " << label_frame_id << '\n';
  }

  // Export and render the glTF scene.
  SetGltfCameraPerspective(
      pipelines_[ImageType::kLabel]->renderer->GetActiveCamera(),
      camera.core());
  const std::string scene_path = ExportScene(ImageType::kLabel, label_frame_id);
  if (verbose()) {
    std::cout << "  Scene exported to: " << scene_path << '\n';
  }

  const std::string img_path =
      UploadAndRender(camera.core(), ImageType::kLabel, scene_path);
  if (verbose()) {
    std::cout << "  Server response image: " << img_path << '\n';
  }

  // Load the returned image back to the drake buffer.
  LoadLabelImage(img_path, label_image_out);
  ++label_frame_id;
}

std::string RenderClientGLTF::ExportPathFor(ImageType image_type,
                                            size_t frame_id) const {
  // Create e.g., {temp_directory()}/000000000000-color.gltf
  const drake::filesystem::path base{temp_directory()};
  const std::string frame{fmt::format("{:0>12}", frame_id)};
  std::string suffix;
  if (image_type == ImageType::kColor)
    suffix = "-color.gltf";
  else if (image_type == ImageType::kDepth)
    suffix = "-depth.gltf";
  else  // image_type == ImageType::kLabel
    suffix = "-label.gltf";
  return base / (frame + suffix);
}

std::string RenderClientGLTF::ExportScene(ImageType image_type,
                                          size_t frame_id) const {
  vtkNew<vtkGLTFExporter> gltf_exporter;
  gltf_exporter->InlineDataOn();
  gltf_exporter->SetRenderWindow(pipelines_[image_type]->window);
  const std::string scene_path = ExportPathFor(image_type, frame_id);
  gltf_exporter->SetFileName(scene_path.c_str());
  gltf_exporter->Write();
  return scene_path;
}

std::string RenderClientGLTF::UploadAndRender(const RenderCameraCore& core,
                                              ImageType image_type,
                                              const std::string& scene_path,
                                              double min_depth,
                                              double max_depth) const {
  if (image_type == ImageType::kDepth)
    ValidDepthRangeOrThrow(min_depth, max_depth);

  const std::string scene_sha256 = ComputeSha256(scene_path);
  UploadScene(image_type, scene_path, scene_sha256);
  return RetrieveRender(core, image_type, scene_path, scene_sha256, min_depth,
                        max_depth);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake

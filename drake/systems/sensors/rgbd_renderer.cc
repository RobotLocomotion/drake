#include "drake/systems/sensors/rgbd_renderer.h"

#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"

// TODO(kunimatsu-tri) Refactor RgbdRenderer with GeometryWorld when it's ready,
// so that other VTK dependent sensor simulators can share the world without
// duplicating it.

namespace drake {
namespace systems {
namespace sensors {

using vtk_util::ConvertToVtkTransform;
using vtk_util::MakeVtkPointerArray;

namespace {
const int kNumMaxLabel = 256;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;

// For Zbuffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_ABORT_MSG("File has no extention.");
  }
  return filepath.substr(0, last_dot);
}

// Updates VTK rendering related objects including vtkRenderWindow,
// vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
// vtkActors' pose update for rendering.
void PerformVTKUpdate(
    const vtkNew<vtkRenderWindow>& window,
    const vtkNew<vtkWindowToImageFilter>& filter,
    const vtkNew<vtkImageExport>& exporter) {
  window->Render();
  filter->Modified();
  filter->Update();
  exporter->Update();
}

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down. For the detail, please refere
  // to CameraInfo's document.
  camera->ApplyTransform(X_WC);
}

}  // namespace

RgbdRenderer::RgbdRenderer(const Eigen::Isometry3d& X_WC,
                           int width, int height,
                           double z_near, double z_far,
                           double fov_y, bool show_window)
    : width_(width), height_(height), fov_y_(fov_y),
      z_near_(z_near), z_far_(z_far),
      color_palette_(kNumMaxLabel, Label::kFlatTerrain, Label::kNoBody) {
  if (!show_window) {
    for (auto& window : MakeVtkPointerArray(color_depth_render_window_,
                                            label_render_window_)) {
      window->SetOffScreenRendering(1);
    }
  }

  const auto sky_color =
      ColorPalette::Normalize(color_palette_.get_sky_color());
  const auto renderers = MakeVtkPointerArray(color_depth_renderer_,
                                             label_renderer_);
  const vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (auto& renderer : renderers) {
    renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
    auto camera = renderer->GetActiveCamera();
    camera->SetViewAngle(fov_y_ * 180. / M_PI);
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }

#if ((VTK_MAJOR_VERSION == 7) && (VTK_MINOR_VERSION >= 1)) || \
    (VTK_MAJOR_VERSION >= 8)
  color_depth_renderer_->SetUseDepthPeeling(1);
  color_depth_renderer_->UseFXAAOn();
#endif

  const auto windows = MakeVtkPointerArray(
      color_depth_render_window_, label_render_window_);
  for (size_t i = 0; i < windows.size(); ++i) {
    windows[i]->SetSize(width_, height_);
    windows[i]->AddRenderer(renderers[i].GetPointer());
  }
  label_render_window_->SetMultiSamples(0);

  color_filter_->SetInput(color_depth_render_window_.GetPointer());
  color_filter_->SetInputBufferTypeToRGBA();
  depth_filter_->SetInput(color_depth_render_window_.GetPointer());
  depth_filter_->SetInputBufferTypeToZBuffer();
  label_filter_->SetInput(label_render_window_.GetPointer());
  label_filter_->SetInputBufferTypeToRGB();

  auto exporters = MakeVtkPointerArray(
      color_exporter_, depth_exporter_, label_exporter_);

  auto filters = MakeVtkPointerArray(
      color_filter_, depth_filter_, label_filter_);

  for (int i = 0; i < 3; ++i) {
    filters[i]->SetMagnification(1);
    filters[i]->ReadFrontBufferOff();
    filters[i]->Update();
#if VTK_MAJOR_VERSION <= 5
    exporters[i]->SetInput(filters[i]->GetOutput());
#else
    exporters[i]->SetInputData(filters[i]->GetOutput());
#endif
    exporters[i]->ImageLowerLeftOff();
  }
}

optional<RgbdRenderer::VisualIndex> RgbdRenderer::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  // Initializes containers in id_object_maps_ if it's not done.
  for (auto& id_object_map : id_object_maps_) {
    const auto it = id_object_map.find(body_id);
    if (it == id_object_map.end()) {
      std::vector<vtkSmartPointer<vtkActor>> vec;
      id_object_map[body_id] = vec;
    }
  }

  vtkNew<vtkActor> actor;
  vtkNew<vtkPolyDataMapper> mapper;
  bool shape_matched = true;
  bool texture_found = false;
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  switch (visual.getShape()) {
    case DrakeShapes::BOX: {
      auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      vtkNew<vtkCubeSource> vtk_cube;
      vtk_cube->SetXLength(box.size(0));
      vtk_cube->SetYLength(box.size(1));
      vtk_cube->SetZLength(box.size(2));

      mapper->SetInputConnection(vtk_cube->GetOutputPort());
      break;
    }
    case DrakeShapes::SPHERE: {
      auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      vtkNew<vtkSphereSource> vtk_sphere;
      vtk_sphere->SetRadius(sphere.radius);
      vtk_sphere->SetThetaResolution(50);
      vtk_sphere->SetPhiResolution(50);

      mapper->SetInputConnection(vtk_sphere->GetOutputPort());
      break;
    }
    case DrakeShapes::CYLINDER: {
      auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      vtkNew<vtkCylinderSource> vtk_cylinder;
      vtk_cylinder->SetHeight(cylinder.length);
      vtk_cylinder->SetRadius(cylinder.radius);
      vtk_cylinder->SetResolution(50);

      // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
      // to rotate it to be z-axis aligned because that is what Drake uses.
      vtkNew<vtkTransform> transform;
      transform->RotateX(90);
      vtkNew<vtkTransformPolyDataFilter> transform_filter;
      transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
      transform_filter->SetTransform(transform.GetPointer());
      transform_filter->Update();

      mapper->SetInputConnection(transform_filter->GetOutputPort());
      break;
    }
    case DrakeShapes::MESH: {
      const auto* mesh_filename = dynamic_cast<const DrakeShapes::Mesh&>(
          geometry).resolved_filename_.c_str();

      // TODO(kunimatsu-tri) Add support for other file formats.
      vtkNew<vtkOBJReader> mesh_reader;
      mesh_reader->SetFileName(mesh_filename);
      mesh_reader->Update();

      // TODO(kunimatsu-tri) Guessing the texture file name is bad. Instead,
      // get it from somewhere like `DrakeShapes::MeshWithTexture` when it's
      // implemented.
      // TODO(kunimatsu-tri) Add support for other file formats.
      const std::string texture_file(
          RemoveFileExtension(mesh_filename) + ".png");
      std::ifstream file_exist(texture_file);

      if (file_exist) {
        vtkNew<vtkPNGReader> texture_reader;
        texture_reader->SetFileName(texture_file.c_str());
        texture_reader->Update();
        vtkNew<vtkTexture> texture;
        texture->SetInputConnection(texture_reader->GetOutputPort());
        texture->InterpolateOn();
        actor->SetTexture(texture.GetPointer());

        texture_found = true;
      }

      mapper->SetInputConnection(mesh_reader->GetOutputPort());
      break;
    }
    case DrakeShapes::CAPSULE: {
      // TODO(kunimatsu-tri) Implement this as needed.
      shape_matched = false;
      break;
    }
    default: {
      shape_matched = false;
      break;
    }
  }

  // Registers actors.
  if (shape_matched) {
    if (!texture_found) {
      const auto color = visual.getMaterial();
      actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    }

    const auto& color = color_palette_.get_normalized_color(body_id);
    vtkNew<vtkActor> actor_for_label;
    actor_for_label->GetProperty()->SetColor(color.r, color.g, color.b);
    // This is to disable shadows and to get an object painted with a single
    // color.
    actor_for_label->GetProperty()->LightingOff();

    vtkSmartPointer<vtkTransform> vtk_transform =
        ConvertToVtkTransform(visual.getWorldTransform());

    auto renderers = MakeVtkPointerArray(color_depth_renderer_,
                                         label_renderer_);
    auto actors = MakeVtkPointerArray(actor, actor_for_label);

    for (size_t i = 0; i < actors.size(); ++i) {
      actors[i]->SetMapper(mapper.GetPointer());
      actors[i]->SetUserTransform(vtk_transform);
      renderers[i]->AddActor(actors[i].GetPointer());
      id_object_maps_[i][body_id].push_back(
          vtkSmartPointer<vtkActor>(actors[i].GetPointer()));
    }

    return optional<VisualIndex>(VisualIndex(
        static_cast<int>(id_object_maps_[0][body_id].size() - 1)));
  }

  return nullopt;
}

void RgbdRenderer::AddFlatTerrain() {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtk_util::CreateSquarePlane(kTerrainSize);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());

  auto color = ColorPalette::Normalize(color_palette_.get_terrain_color());
  terrain_actor_->GetProperty()->SetColor(color.r, color.g, color.b);
  terrain_actor_->GetProperty()->LightingOff();
  for (auto& renderer : MakeVtkPointerArray(color_depth_renderer_,
                                            label_renderer_)) {
    renderer->AddActor(terrain_actor_.GetPointer());
  }
}

void RgbdRenderer::UpdateViewpoint(const Eigen::Isometry3d& X_WR) const {
  vtkSmartPointer<vtkTransform> vtk_X_WR = ConvertToVtkTransform(X_WR);

  for (auto& renderer : MakeVtkPointerArray(color_depth_renderer_,
                                            label_renderer_)) {
    auto camera = renderer->GetActiveCamera();
    // TODO(kunimatsu-tri) Once VTK 5.8 support dropped, rewrite this
    // using `vtkCamera`'s `SetModelTransformMatrix` method which is
    // introduced since VTK 5.10.
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WR);
  }
}

void RgbdRenderer::UpdateVisualPose(const Eigen::Isometry3d& X_WV,
                                    int body_id,
                                    VisualIndex visual_id) const {
  vtkSmartPointer<vtkTransform> vtk_X_WV = ConvertToVtkTransform(X_WV);
  // `id_object_maps_` is modified here. This is OK because 1) we are just
  // copying data to the memory spaces allocated at construction time
  // and 2) we are not outputting these data to outside the class.
  for (auto& id_object_map : id_object_maps_) {
    auto& actor = id_object_map.at(body_id).at(visual_id);
    actor->SetUserTransform(vtk_X_WV);
  }
}

void RgbdRenderer::RenderColorImage(ImageRgba8U* color_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(color_depth_render_window_, color_filter_, color_exporter_);
  color_exporter_->Export(color_image_out->at(0, 0));
}

void RgbdRenderer::RenderDepthImage(ImageDepth32F* depth_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(color_depth_render_window_, depth_filter_, depth_exporter_);
  depth_exporter_->Export(depth_image_out->at(0, 0));

  // TODO(kunimatsu-tri) Calculate this in a vertex shader.
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      depth_image_out->at(u, v)[0] =
          CheckRangeAndConvertToMeters(depth_image_out->at(u, v)[0]);
    }
  }
}

void RgbdRenderer::RenderLabelImage(ImageLabel16I* label_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(label_render_window_, label_filter_, label_exporter_);

  ImageRgb8U image(width_, height_);
  label_exporter_->Export(image.at(0, 0));

  ColorI color;
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      label_image_out->at(u, v)[0] =
          static_cast<int16_t>(color_palette_.LookUpId(color));
    }
  }
}

float RgbdRenderer::CheckRangeAndConvertToMeters(float z_buffer_value) const {
  float z;
  // When the depth is either closer than `kClippingPlaneNear` or farther than
  // `kClippingPlaneFar`, `z_buffer_value` becomes `1.f`.
  if (z_buffer_value == 1.f) {
    z = std::numeric_limits<float>::quiet_NaN();
  } else {
    z = static_cast<float>(kB / (z_buffer_value - kA));

    if (z > z_far_) {
      z = RgbdRenderer::InvalidDepth::kTooFar;
    } else if (z < z_near_) {
      z = RgbdRenderer::InvalidDepth::kTooClose;
    }
  }

  return z;
}

constexpr float RgbdRenderer::InvalidDepth::kTooFar;
constexpr float RgbdRenderer::InvalidDepth::kTooClose;

constexpr int16_t RgbdRenderer::Label::kNoBody;
constexpr int16_t RgbdRenderer::Label::kFlatTerrain;

}  // namespace sensors
}  // namespace systems
}  // namespace drake

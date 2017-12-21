#include "drake/systems/sensors/rgbd_renderer_vtk.h"

#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkAutoInit.h>
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
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/vtk_util.h"

// This macro declares vtkRenderingOpenGL2_AutoInit_{Construct(), Destruct()}
// functions and the former is called via VTK_AUTOINIT_CONSTRUCT in
// ModuleInitVtkRenderingOpenGL2.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)

// TODO(kunimatsu-tri) Refactor RgbdRenderer with GeometryWorld when it's ready,
// so that other VTK dependent sensor simulators can share the world without
// duplicating it.

namespace drake {
namespace systems {
namespace sensors {

using vtk_util::ConvertToVtkTransform;
using vtk_util::MakeVtkPointerArray;

namespace {

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;

// For Zbuffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

// Register the object factories for the vtkRenderingOpenGL2 module.
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

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
void PerformVTKUpdate(const vtkNew<vtkRenderWindow>& window,
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
  camera->SetViewUp(0., -1, 0.);  // Sets y-down. For the detail, please refer
  // to CameraInfo's document.
  camera->ApplyTransform(X_WC);
}

}  // namespace

class RgbdRendererVTK::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererVTK* parent, const Eigen::Isometry3d& X_WC);
  ~Impl() {}

  void ImplAddFlatTerrain();

  optional<VisualIndex> ImplRegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id);

  void ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV, int body_id,
                          VisualIndex visual_id) const;

  void ImplUpdateViewpoint(const Eigen::Isometry3d& X_WC) const;

  void ImplRenderColorImage(ImageRgba8U* color_image_out) const;

  void ImplRenderDepthImage(ImageDepth32F* depth_image_out) const;

  void ImplRenderLabelImage(ImageLabel16I* label_image_out) const;

 private:
  float CheckRangeAndConvertToMeters(float z_buffer_value) const;

  RgbdRendererVTK* parent_ = nullptr;

  vtkNew<vtkActor> terrain_actor_;
  // An array of maps which take pairs of a body index in RBT and a vector of
  // vtkSmartPointer to vtkActor. The each vtkActor corresponds to an visual
  // element specified in SDF / URDF. The first element of this array is for
  // color and depth rendering and the second is for label image rendering.
  // TODO(kunimatsu-tri) Make this more straight forward for the readability.
  std::array<std::map<int, std::vector<vtkSmartPointer<vtkActor>>>, 2>
      id_object_maps_;
  vtkNew<vtkRenderer> color_depth_renderer_;
  vtkNew<vtkRenderer> label_renderer_;
  vtkNew<vtkRenderWindow> color_depth_render_window_;
  vtkNew<vtkRenderWindow> label_render_window_;
  vtkNew<vtkWindowToImageFilter> color_filter_;
  vtkNew<vtkWindowToImageFilter> depth_filter_;
  vtkNew<vtkWindowToImageFilter> label_filter_;
  vtkNew<vtkImageExport> color_exporter_;
  vtkNew<vtkImageExport> depth_exporter_;
  vtkNew<vtkImageExport> label_exporter_;
};

float RgbdRendererVTK::Impl::CheckRangeAndConvertToMeters(
    float z_buffer_value) const {
  float z;
  // When the depth is either closer than `kClippingPlaneNear` or farther than
  // `kClippingPlaneFar`, `z_buffer_value` becomes `1.f`.
  if (z_buffer_value == 1.f) {
    z = std::numeric_limits<float>::quiet_NaN();
  } else {
    z = static_cast<float>(kB / (z_buffer_value - kA));

    if (z > parent_->config().z_far) {
      z = InvalidDepth::kTooFar;
    } else if (z < parent_->config().z_near) {
      z = InvalidDepth::kTooClose;
    }
  }

  return z;
}

void RgbdRendererVTK::Impl::ImplAddFlatTerrain() {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtk_util::CreateSquarePlane(kTerrainSize);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());

  auto color =
      ColorPalette::Normalize(parent_->color_palette().get_terrain_color());
  terrain_actor_->GetProperty()->SetColor(color.r, color.g, color.b);
  terrain_actor_->GetProperty()->LightingOff();
  for (auto& renderer :
       MakeVtkPointerArray(color_depth_renderer_, label_renderer_)) {
    renderer->AddActor(terrain_actor_.GetPointer());
  }
}

void RgbdRendererVTK::Impl::ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV,
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

void RgbdRendererVTK::Impl::ImplUpdateViewpoint(
    const Eigen::Isometry3d& X_WC) const {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (auto& renderer :
       MakeVtkPointerArray(color_depth_renderer_, label_renderer_)) {
    auto camera = renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RgbdRendererVTK::Impl::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(color_depth_render_window_, color_filter_, color_exporter_);
  color_exporter_->Export(color_image_out->at(0, 0));
}

void RgbdRendererVTK::Impl::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(color_depth_render_window_, depth_filter_, depth_exporter_);
  depth_exporter_->Export(depth_image_out->at(0, 0));

  // TODO(kunimatsu-tri) Calculate this in a vertex shader.
  for (int v = 0; v < parent_->config().height; ++v) {
    for (int u = 0; u < parent_->config().width; ++u) {
      depth_image_out->at(u, v)[0] =
          CheckRangeAndConvertToMeters(depth_image_out->at(u, v)[0]);
    }
  }
}

void RgbdRendererVTK::Impl::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(label_render_window_, label_filter_, label_exporter_);

  ImageRgb8U image(parent_->config().width, parent_->config().height);
  label_exporter_->Export(image.at(0, 0));

  ColorI color;
  for (int v = 0; v < parent_->config().height; ++v) {
    for (int u = 0; u < parent_->config().width; ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      label_image_out->at(u, v)[0] =
          static_cast<int16_t>(parent_->color_palette().LookUpId(color));
    }
  }
}

RgbdRendererVTK::Impl::Impl(RgbdRendererVTK* parent,
                            const Eigen::Isometry3d& X_WC)
    : parent_(parent) {
  if (!parent_->config().show_window) {
    for (auto& window : MakeVtkPointerArray(color_depth_render_window_,
                                            label_render_window_)) {
      window->SetOffScreenRendering(1);
    }
  }

  const auto sky_color =
      ColorPalette::Normalize(parent_->color_palette().get_sky_color());
  const auto renderers =
      MakeVtkPointerArray(color_depth_renderer_, label_renderer_);
  const vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (auto& renderer : renderers) {
    renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
    auto camera = renderer->GetActiveCamera();
    camera->SetViewAngle(parent_->config().fov_y * 180. / M_PI);
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }

  color_depth_renderer_->SetUseDepthPeeling(1);
  color_depth_renderer_->UseFXAAOn();

  const auto windows =
      MakeVtkPointerArray(color_depth_render_window_, label_render_window_);
  for (size_t i = 0; i < windows.size(); ++i) {
    windows[i]->SetSize(parent_->config().width, parent_->config().height);
    windows[i]->AddRenderer(renderers[i].GetPointer());
  }
  label_render_window_->SetMultiSamples(0);

  color_filter_->SetInput(color_depth_render_window_.GetPointer());
  color_filter_->SetInputBufferTypeToRGBA();
  depth_filter_->SetInput(color_depth_render_window_.GetPointer());
  depth_filter_->SetInputBufferTypeToZBuffer();
  label_filter_->SetInput(label_render_window_.GetPointer());
  label_filter_->SetInputBufferTypeToRGB();

  auto exporters =
      MakeVtkPointerArray(color_exporter_, depth_exporter_, label_exporter_);

  auto filters =
      MakeVtkPointerArray(color_filter_, depth_filter_, label_filter_);

  for (int i = 0; i < 3; ++i) {
    filters[i]->SetMagnification(1);
    filters[i]->ReadFrontBufferOff();
    filters[i]->Update();
    exporters[i]->SetInputData(filters[i]->GetOutput());
    exporters[i]->ImageLowerLeftOff();
  }
}

optional<RgbdRenderer::VisualIndex> RgbdRendererVTK::Impl::ImplRegisterVisual(
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
      const auto* mesh_filename =
          dynamic_cast<const DrakeShapes::Mesh&>(geometry)
              .resolved_filename_.c_str();

      // TODO(kunimatsu-tri) Add support for other file formats.
      vtkNew<vtkOBJReader> mesh_reader;
      mesh_reader->SetFileName(mesh_filename);
      mesh_reader->Update();

      // TODO(kunimatsu-tri) Guessing the texture file name is bad. Instead,
      // get it from somewhere like `DrakeShapes::MeshWithTexture` when it's
      // implemented.
      // TODO(kunimatsu-tri) Add support for other file formats.
      const std::string texture_file(RemoveFileExtension(mesh_filename) +
                                     ".png");
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

    const auto& color = parent_->color_palette().get_normalized_color(body_id);
    vtkNew<vtkActor> actor_for_label;
    actor_for_label->GetProperty()->SetColor(color.r, color.g, color.b);
    // This is to disable shadows and to get an object painted with a single
    // color.
    actor_for_label->GetProperty()->LightingOff();

    vtkSmartPointer<vtkTransform> vtk_transform =
        ConvertToVtkTransform(visual.getWorldTransform());

    auto renderers =
        MakeVtkPointerArray(color_depth_renderer_, label_renderer_);
    auto actors = MakeVtkPointerArray(actor, actor_for_label);

    for (size_t i = 0; i < actors.size(); ++i) {
      actors[i]->SetMapper(mapper.GetPointer());
      actors[i]->SetUserTransform(vtk_transform);
      renderers[i]->AddActor(actors[i].GetPointer());
      id_object_maps_[i][body_id].push_back(
          vtkSmartPointer<vtkActor>(actors[i].GetPointer()));
    }

    return optional<VisualIndex>(
        VisualIndex(static_cast<int>(id_object_maps_[0][body_id].size() - 1)));
  }

  return nullopt;
}

RgbdRendererVTK::RgbdRendererVTK(const RenderingConfig& config,
                                 const Eigen::Isometry3d& X_WC)
    : RgbdRenderer(config, X_WC),
      impl_(new RgbdRendererVTK::Impl(this, X_WC)) {}

RgbdRendererVTK::~RgbdRendererVTK() {}

optional<RgbdRenderer::VisualIndex> RgbdRendererVTK::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return impl_->ImplRegisterVisual(visual, body_id);
}

void RgbdRendererVTK::ImplAddFlatTerrain() { impl_->ImplAddFlatTerrain(); }

void RgbdRendererVTK::ImplUpdateViewpoint(const Eigen::Isometry3d& X_WC) const {
  impl_->ImplUpdateViewpoint(X_WC);
}

void RgbdRendererVTK::ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV,
                                       int body_id,
                                       VisualIndex visual_id) const {
  impl_->ImplUpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRendererVTK::ImplRenderColorImage(ImageRgba8U* color_image_out) const {
  impl_->ImplRenderColorImage(color_image_out);
}

void RgbdRendererVTK::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  impl_->ImplRenderDepthImage(depth_image_out);
}

void RgbdRendererVTK::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  impl_->ImplRenderLabelImage(label_image_out);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

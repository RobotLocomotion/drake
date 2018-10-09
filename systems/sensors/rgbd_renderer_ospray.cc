#include "drake/systems/sensors/rgbd_renderer_ospray.h"

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
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageExport.h>
#include <vtkJPEGReader.h>
#include <vtkLight.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkOSPRayLightNode.h>
#include <vtkOSPRayMaterialLibrary.h>
#include <vtkOSPRayPass.h>
#include <vtkOSPRayRendererNode.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/systems/sensors/depth_shaders.h"
#include "drake/systems/sensors/vtk_util.h"

VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)

// TODO(kunimatsu-tri) Refactor RgbdRenderer with SceneGraph when it's
// ready, so that other VTK dependent sensor simulators can share the world
// without duplicating it.

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

enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

const int kNumOutputImage = 1;

struct RenderingPipeline {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  vtkNew<vtkWindowToImageFilter> filter;
  vtkNew<vtkImageExport> exporter;
};

using ActorCollection = std::vector<vtkSmartPointer<vtkActor>>;

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_ABORT_MSG("File has no extension.");
  }
  return filepath.substr(0, last_dot);
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

struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

}  // namespace

class RgbdRendererOSPRay::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererOSPRay* parent, const Eigen::Isometry3d& X_WC);
  ~Impl() {}

  void SetBackground(const std::string& filepath);

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
  RgbdRendererOSPRay* parent_ = nullptr;
  vtkNew<vtkLight> light_;
  vtkNew<vtkOSPRayMaterialLibrary> materials_;
  vtkNew<vtkActor> terrain_actor_;
  // Use ImageType to access to this array. We assume pipelines_'s indices to be
  // 0 for RGB, 1 for depth, and 2 for ground-truth label rendering.
  std::array<std::unique_ptr<RenderingPipeline>,
             kNumOutputImage> pipelines_;

  // A map which takes pairs of a body index in RBT and three vectors of
  // vtkSmartPointer to vtkActor for color, depth and label rendering
  // respectively. Each vtkActor corresponds to an visual element specified in
  // SDF / URDF.
  std::map<int, std::array<ActorCollection, kNumOutputImage>> id_object_maps_;

  vtkNew<vtkOSPRayPass> ospray_;
  std::array<vtkNew<vtkActor>, kNumOutputImage> actors_;
  std::array<vtkNew<vtkPolyDataMapper>, kNumOutputImage> mappers_;
};

void RgbdRendererOSPRay::Impl::SetBackground(
    const std::string& filepath) {
  std::ifstream file_exist(filepath);
  if (!file_exist) {
    throw std::runtime_error("Background file not found.");
  }
  auto ext = filepath.substr(filepath.find_last_of(".") + 1);
  if (ext != "jpg" && ext != "jpeg") {
    throw std::runtime_error("Background file is not jpeg nor jpg.");
  }

  vtkNew<vtkJPEGReader> jpg_reader;
  jpg_reader->SetFileName(filepath.c_str());
  jpg_reader->Update();
  vtkNew<vtkTexture> tex;
  tex->SetInputConnection(jpg_reader->GetOutputPort(0));
  pipelines_[ImageType::kColor]->renderer->TexturedBackgroundOn();
  pipelines_[ImageType::kColor]->renderer->SetBackgroundTexture(tex);
}

void RgbdRendererOSPRay::Impl::ImplAddFlatTerrain() {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtk_util::CreateSquarePlane(kTerrainSize);

  materials_->AddMaterial("terrain", "MetallicPaint");
  // For color.
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());
  auto color = ColorPalette<int>::Normalize(
      parent_->color_palette().get_terrain_color());

  terrain_actor_->GetProperty()->SetColor(color.r, color.g, color.b);
  terrain_actor_->GetProperty()->LightingOff();
  auto prop = terrain_actor_->GetProperty();
  prop->SetMaterialName("terrain");

  pipelines_[ImageType::kColor]->renderer->AddActor(
      terrain_actor_.GetPointer());
}

void RgbdRendererOSPRay::Impl::ImplUpdateVisualPose(
    const Eigen::Isometry3d& X_WV,
    int body_id,
    VisualIndex visual_id) const {
  vtkSmartPointer<vtkTransform> vtk_X_WV = ConvertToVtkTransform(X_WV);
  // `id_object_maps_` is modified here. This is OK because 1) we are just
  // copying data to the memory spaces allocated at construction time
  // and 2) we are not outputting these data to outside the class.
  auto& actor_collections = id_object_maps_.at(body_id);
  for (auto& actor_collection : actor_collections) {
    actor_collection.at(visual_id)->SetUserTransform(vtk_X_WV);
  }
}

void RgbdRendererOSPRay::Impl::ImplUpdateViewpoint(
    const Eigen::Isometry3d& X_WC) const {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  light_->SetTransformMatrix(vtk_X_WC->GetMatrix());

  for (auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RgbdRendererOSPRay::Impl::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  auto& p = pipelines_[ImageType::kColor];
  p->window->Render();
  vtkRenderer* renderer = p->window->GetRenderers()->GetFirstRenderer();
  int max_frames = vtkOSPRayRendererNode::GetMaxFrames(renderer);

  for (int i = 0; i < max_frames; ++i) {
    p->window->Render();
  }
  p->filter->Modified();
  p->filter->Update();
  p->exporter->Update();
  p->exporter->Export(color_image_out->at(0, 0));
}

void RgbdRendererOSPRay::Impl::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  // TODO(kunimatsu-tri) Implement this.
  drake::unused(depth_image_out);
  throw std::runtime_error("Not implemented.");
}

void RgbdRendererOSPRay::Impl::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  // TODO(kunimatsu-tri) Implement this.
  drake::unused(label_image_out);
  throw std::runtime_error("Not implemented.");
}

RgbdRendererOSPRay::Impl::Impl(RgbdRendererOSPRay* parent,
                            const Eigen::Isometry3d& X_WC)
    : parent_(parent),
      pipelines_{{std::make_unique<RenderingPipeline>()}} {
  auto& cp = pipelines_[ImageType::kColor];
  if (parent_->config().show_window) {
    cp->window->SetWindowName("Color Image");
  } else {
    for (auto& pipeline : pipelines_) {
      pipeline->window->SetOffScreenRendering(1);
    }
  }

  // OSPRay specific configuration.
  cp->renderer->SetPass(ospray_);
  vtkOSPRayRendererNode::SetRendererType("pathtracer", cp->renderer);
  vtkOSPRayRendererNode::SetSamplesPerPixel(1, cp->renderer);

  double np[3] = {0, 0, 1};
  double ep[3] = {0, 1, 0};
  vtkOSPRayRendererNode::SetNorthPole(np, cp->renderer);
  vtkOSPRayRendererNode::SetEastPole(ep, cp->renderer);
  vtkOSPRayRendererNode::SetMaterialLibrary(materials_, cp->renderer);
  vtkOSPRayRendererNode::SetMaxFrames(10, cp->renderer);

  const vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(parent_->config().fov_y * 180. / M_PI);
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
    pipeline->window->SetSize(parent_->config().width,
                              parent_->config().height);
    pipeline->window->AddRenderer(pipeline->renderer);
    pipeline->filter->SetInput(pipeline->window);
    pipeline->filter->SetScale(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->filter->Update();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();
  }

  // TODO(kunimatsu-tri) Add API to handle lighting stuff.
  light_->SetPosition(-2, 0, 10);
  light_->SetFocalPoint(0, 0, 0);
  light_->PositionalOff();
  // OSPRay specific control, radius to get soft shadows.
  vtkOSPRayLightNode::SetRadius(2.0, light_);
  light_->SetTransformMatrix(vtk_X_WC->GetMatrix());

  cp->renderer->AddLight(light_);
}

optional<RgbdRenderer::VisualIndex>
RgbdRendererOSPRay::Impl::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  bool shape_matched = true;
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  switch (visual.getShape()) {
    // TODO(kunimatsu-tri) Load material property for primitive shapes.
    // Material properties can be loaded only for mesh at this point.
    case DrakeShapes::BOX: {
      const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      vtkNew<vtkCubeSource> vtk_cube;
      vtk_cube->SetXLength(box.size(0));
      vtk_cube->SetYLength(box.size(1));
      vtk_cube->SetZLength(box.size(2));

      for (auto& mapper : mappers_) {
        mapper->SetInputConnection(vtk_cube->GetOutputPort());
      }
      break;
    }
    case DrakeShapes::SPHERE: {
      const auto& sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      vtkNew<vtkSphereSource> vtk_sphere;
      vtk_sphere->SetRadius(sphere.radius);
      vtk_sphere->SetThetaResolution(50);
      vtk_sphere->SetPhiResolution(50);

      for (auto& mapper : mappers_) {
        mapper->SetInputConnection(vtk_sphere->GetOutputPort());
      }
      break;
    }
    case DrakeShapes::CYLINDER: {
      const auto& cylinder =
          dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
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

      for (auto& mapper : mappers_) {
        mapper->SetInputConnection(transform_filter->GetOutputPort());
      }
      break;
    }
    case DrakeShapes::MESH: {
      const auto& mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
      const auto* mesh_filename = mesh.resolved_filename_.c_str();

      // TODO(kunimatsu-tri) Add support for other file formats.
      vtkNew<vtkOBJReader> mesh_reader;
      mesh_reader->SetFileName(mesh_filename);
      mesh_reader->Update();

      // Changing the scale of the loaded mesh.
      const double scale_x = mesh.scale_[0];
      const double scale_y = mesh.scale_[1];
      const double scale_z = mesh.scale_[2];
      vtkNew<vtkTransform> transform;
      transform->Scale(scale_x, scale_y, scale_z);
      vtkNew<vtkTransformPolyDataFilter> transform_filter;
      transform_filter->SetInputConnection(mesh_reader->GetOutputPort());
      transform_filter->SetTransform(transform.GetPointer());
      transform_filter->Update();

      for (auto& mapper : mappers_) {
        mapper->SetInputConnection(transform_filter->GetOutputPort());
      }

      // TODO(kunimatsu-tri) Guessing the material file name is bad. Instead,
      // load .mtl file referred from the .obj file when
      // vtkOSPRayMaterialLibrary::ReadFile for .mtl is released.
      const std::string file = RemoveFileExtension(mesh_filename);
      bool success = materials_->ReadFile(std::string(file + ".json").c_str());
      if (success) {
        auto prop = actors_[ImageType::kColor]->GetProperty();
        auto f = file.substr(file.find_last_of("/") + 1);
        prop->SetMaterialName(f.c_str());
      } else {
        throw std::runtime_error("Found no material file.");
      }
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
    auto& color_actor = actors_[ImageType::kColor];
    if (color_actor->GetProperty()->GetNumberOfTextures() == 0) {
      // Taken from diffuse color.
      const auto color = visual.getMaterial();
      color_actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    }

    vtkSmartPointer<vtkTransform> vtk_transform =
        ConvertToVtkTransform(visual.getWorldTransform());
    auto& actor_collections = id_object_maps_[body_id];
    for (size_t i = 0; i < actors_.size(); ++i) {
      actors_[i]->SetMapper(mappers_[i].GetPointer());
      actors_[i]->SetUserTransform(vtk_transform);
      pipelines_[i]->renderer->AddActor(actors_[i].GetPointer());
      actor_collections[i].push_back(actors_[i].GetPointer());
    }

    return optional<VisualIndex>(VisualIndex(static_cast<int>(
        id_object_maps_[body_id][ImageType::kColor].size() - 1)));
  }

  return nullopt;
}

RgbdRendererOSPRay::RgbdRendererOSPRay(const RenderingConfig& config,
                                       const Eigen::Isometry3d& X_WC)
    : RgbdRenderer(config, X_WC),
      impl_(new RgbdRendererOSPRay::Impl(this, X_WC)) {}

RgbdRendererOSPRay::~RgbdRendererOSPRay() {}

optional<RgbdRenderer::VisualIndex> RgbdRendererOSPRay::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return impl_->ImplRegisterVisual(visual, body_id);
}

void RgbdRendererOSPRay::ImplAddFlatTerrain() { impl_->ImplAddFlatTerrain(); }

void RgbdRendererOSPRay::ImplUpdateViewpoint(
    const Eigen::Isometry3d& X_WC) const {
  impl_->ImplUpdateViewpoint(X_WC);
}

void RgbdRendererOSPRay::ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV,
                                              int body_id,
                                              VisualIndex visual_id) const {
  impl_->ImplUpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRendererOSPRay::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
  impl_->ImplRenderColorImage(color_image_out);
}

void RgbdRendererOSPRay::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  impl_->ImplRenderDepthImage(depth_image_out);
}

void RgbdRendererOSPRay::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  impl_->ImplRenderLabelImage(label_image_out);
}

void RgbdRendererOSPRay::SetBackground(const std::string& filepath) {
  impl_->SetBackground(filepath);
}
}  // namespace sensors
}  // namespace systems
}  // namespace drake

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
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/depth_shaders.h"
#include "drake/systems/sensors/vtk_util.h"

// This macro declares vtkRenderingOpenGL2_AutoInit_{Construct(), Destruct()}
// functions and the former is called via VTK_AUTOINIT_CONSTRUCT in
// ModuleInitVtkRenderingOpenGL2.
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

struct RenderingPipeline {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  vtkNew<vtkWindowToImageFilter> filter;
  vtkNew<vtkImageExport> exporter;
};

using ActorCollection = std::vector<vtkSmartPointer<vtkActor>>;

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
void PerformVTKUpdate(const std::unique_ptr<RenderingPipeline>& p) {
  p->window->Render();
  p->filter->Modified();
  p->filter->Update();
  p->exporter->Update();
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

// A callback class for setting uniform variables used in shader programs,
// namely z_near and z_far, when vtkCommand::UpdateShaderEvent is caught.
// See also shaders::kDepthFS, this is where the variables are used.
// For the detail of VTK's callback mechanism, please refer to:
// https://www.vtk.org/doc/nightly/html/classvtkCommand.html#details
class ShaderCallback : public vtkCommand {
 public:
  static ShaderCallback* New() { return new ShaderCallback; }

  // NOLINTNEXTLINE(runtime/int): To match pre-existing APIs.
  void Execute(vtkObject*, unsigned long, void* callback_object) VTK_OVERRIDE {
    vtkShaderProgram* program =
        reinterpret_cast<vtkShaderProgram*>(callback_object);
    program->SetUniformf("z_near", z_near_);
    program->SetUniformf("z_far", z_far_);
    program = nullptr;
  }

  void set_renderer(vtkRenderer* renderer) { renderer_ = renderer; }

  void set_z_near(float z_near) {
    z_near_ = z_near;
  }

  void set_z_far(float z_far) {
    z_far_ = z_far;
  }

  ShaderCallback() { this->renderer_ = nullptr; }

 private:
  vtkRenderer *renderer_;
  float z_near_{0.f};
  float z_far_{0.f};
};

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
  float CheckRangeAndConvertToMeters(float shader_output) const;

  RgbdRendererVTK* parent_ = nullptr;

  vtkNew<vtkActor> terrain_actor_;
  vtkNew<vtkActor> terrain_depth_actor_;
  // Use ImageType to access to this array. We assume pipelines_'s indices to be
  // 0 for RGB, 1 for depth, and 2 for ground-truth label rendering.
  std::array<std::unique_ptr<RenderingPipeline>, 3> pipelines_;

  // A map which takes pairs of a body index in RBT and three vectors of
  // vtkSmartPointer to vtkActor for color, depth and label rendering
  // respectively. Each vtkActor corresponds to an visual element specified in
  // SDF / URDF.
  std::map<int, std::array<ActorCollection, 3>> id_object_maps_;

  vtkNew<ShaderCallback> uniform_setting_callback_;
};

float RgbdRendererVTK::Impl::CheckRangeAndConvertToMeters(
    float shader_output) const {
  float z;
  const double z_far = parent_->config().z_far;
  const double z_near = parent_->config().z_near;
  const double kA = z_far - kClippingPlaneNear;

  // When the depth is either closer than `kClippingPlaneNear` or farther than
  // `kClippingPlaneFar`, `shader_output` becomes 1.f.
  if (shader_output == 1.f) {
    z = std::numeric_limits<float>::quiet_NaN();
  } else {
    z = static_cast<float>(shader_output * kA + kClippingPlaneNear);

    if (z > z_far) {
      z = InvalidDepth::kTooFar;
    } else if (z < z_near) {
      z = InvalidDepth::kTooClose;
    }
  }

  return z;
}

void RgbdRendererVTK::Impl::ImplAddFlatTerrain() {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtk_util::CreateSquarePlane(kTerrainSize);

  // For color and label.
  vtkNew<vtkOpenGLPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());
  auto color =
      ColorPalette::Normalize(parent_->color_palette().get_terrain_color());

  terrain_actor_->GetProperty()->SetColor(color.r, color.g, color.b);
  terrain_actor_->GetProperty()->LightingOff();

  pipelines_[ImageType::kColor]->renderer->AddActor(
      terrain_actor_.GetPointer());
  pipelines_[ImageType::kLabel]->renderer->AddActor(
      terrain_actor_.GetPointer());

  // For depth.
  vtkNew<vtkOpenGLPolyDataMapper> depth_mapper;
  // Setting a vertex shader program and a fragment shader program here.
  depth_mapper->SetVertexShaderCode(shaders::kDepthVS);
  depth_mapper->SetFragmentShaderCode(shaders::kDepthFS);
  depth_mapper->SetInputConnection(plane->GetOutputPort());
  // Setting a callback for passing uniform variable to the fragment shader
  // program.
  depth_mapper->AddObserver(
      vtkCommand::UpdateShaderEvent, uniform_setting_callback_.Get());

  terrain_depth_actor_->SetMapper(depth_mapper.GetPointer());
  pipelines_[ImageType::kDepth]->renderer->AddActor(
      terrain_depth_actor_.GetPointer());
}


void RgbdRendererVTK::Impl::ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV,
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

void RgbdRendererVTK::Impl::ImplUpdateViewpoint(
    const Eigen::Isometry3d& X_WC) const {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RgbdRendererVTK::Impl::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(pipelines_[ImageType::kColor]);
  pipelines_[ImageType::kColor]->exporter->Export(color_image_out->at(0, 0));
}

void RgbdRendererVTK::Impl::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  const int width = parent_->config().width;
  const int height = parent_->config().height;
  ImageRgba8U image(width, height);

  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(pipelines_[ImageType::kDepth]);
  pipelines_[ImageType::kDepth]->exporter->Export(image.at(0, 0));

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      if (image.at(u, v)[0] == 255u &&
          image.at(u, v)[1] == 255u &&
          image.at(u, v)[2] == 255u) {
        depth_image_out->at(u, v)[0] = InvalidDepth::kTooFar;
      } else {
        // Decoding three channel color values to a float value. For the detail,
        // see depth_shaders.h.
        float shader_value =
            image.at(u, v)[0] +
            image.at(u, v)[1] / 255. +
            image.at(u, v)[2] / (255. * 255.);

        // Dividing by 255 so that the range gets to be [0, 1].
        shader_value /= 255.f;
        // TODO(kunimatsu-tri) Calculate this in a vertex shader.
        depth_image_out->at(u, v)[0] =
            CheckRangeAndConvertToMeters(shader_value);
      }
    }
  }
}

void RgbdRendererVTK::Impl::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  const int width = parent_->config().width;
  const int height = parent_->config().height;
  ImageRgba8U image(width, height);

  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(pipelines_[ImageType::kLabel]);
  pipelines_[ImageType::kLabel]->exporter->Export(image.at(0, 0));

  ColorI color;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      // Converting an RGB color to an object instance ID.
      label_image_out->at(u, v)[0] =
          static_cast<int16_t>(parent_->color_palette().LookUpId(color));
    }
  }
}

RgbdRendererVTK::Impl::Impl(RgbdRendererVTK* parent,
                            const Eigen::Isometry3d& X_WC)
    : parent_(parent),
      pipelines_{{
          std::make_unique<RenderingPipeline>(),
          std::make_unique<RenderingPipeline>(),
          std::make_unique<RenderingPipeline>()}} {
  if (parent_->config().show_window) {
    pipelines_[ImageType::kColor]->window->SetWindowName("Color Image");
    pipelines_[ImageType::kLabel]->window->SetWindowName("Label Image");
    // Always setting off to depth window since displaying the colors which
    // encode floats as depth values doesn't provide useful information to
    // users.
    pipelines_[ImageType::kDepth]->window->SetOffScreenRendering(1);
  } else {
    for (auto& pipeline : pipelines_) {
      pipeline->window->SetOffScreenRendering(1);
    }
  }

  const auto sky_color =
      ColorPalette::Normalize(parent_->color_palette().get_sky_color());
  const vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  pipelines_[ImageType::kLabel]->window->SetMultiSamples(0);
  // Disable multi sampling that has a bug with on-screen rendering
  // with NVidia drivers on Ubuntu 16.04: In certain very specific
  // cases (camera position, scene, triangle drawing order, normal
  // orientation), a plane surface has partial background pixels
  // bleeding through it which changes the color of the center pixel.
  // TODO(fbudin69500) If lack of anti-aliasing in production code is
  // problematic, change this to only disable anti-aliasing in unit
  // tests. Alternatively, find other way to resolve the driver bug.
  pipelines_[ImageType::kColor]->window->SetMultiSamples(0);
  pipelines_[ImageType::kDepth]->window->SetMultiSamples(0);

  for (auto& pipeline : pipelines_) {
    pipeline->renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(parent_->config().fov_y * 180. / M_PI);
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);

    pipeline->window->SetSize(parent_->config().width,
                              parent_->config().height);
    pipeline->window->AddRenderer(pipeline->renderer.GetPointer());
    pipeline->filter->SetInput(pipeline->window.GetPointer());
    pipeline->filter->SetScale(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->filter->Update();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();
  }

  // Differing depth's clipping range from others, so that we can gain
  // precision for depth calculation.
  {
    auto camera = pipelines_[ImageType::kDepth]->renderer->GetActiveCamera();
    camera->SetClippingRange(kClippingPlaneNear, parent_->config().z_far);
    pipelines_[ImageType::kDepth]->renderer->SetBackground(1., 1., 1.);
  }

  pipelines_[ImageType::kColor]->renderer->SetUseDepthPeeling(1);
  pipelines_[ImageType::kColor]->renderer->UseFXAAOn();

  uniform_setting_callback_->set_renderer(
      pipelines_[ImageType::kDepth]->renderer.Get());
  // Setting kClippingPlaneNear instead of z_near_ so that we can distinguish
  // kTooClose from out of range.
  uniform_setting_callback_->set_z_near(kClippingPlaneNear);
  uniform_setting_callback_->set_z_far(
      static_cast<float>(parent_->config().z_far));
}

optional<RgbdRenderer::VisualIndex> RgbdRendererVTK::Impl::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  std::array<vtkNew<vtkActor>, 3> actors;
  std::array<vtkNew<vtkOpenGLPolyDataMapper>, 3> mappers;
  // Sets vertex and fragment shaders only to the depth mapper.
  mappers[ImageType::kDepth]->SetVertexShaderCode(shaders::kDepthVS);
  mappers[ImageType::kDepth]->SetFragmentShaderCode(shaders::kDepthFS);
  mappers[ImageType::kDepth]->AddObserver(
      vtkCommand::UpdateShaderEvent, uniform_setting_callback_.Get());

  bool shape_matched = true;
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  switch (visual.getShape()) {
    case DrakeShapes::BOX: {
      const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      vtkNew<vtkCubeSource> vtk_cube;
      vtk_cube->SetXLength(box.size(0));
      vtk_cube->SetYLength(box.size(1));
      vtk_cube->SetZLength(box.size(2));

      for (auto& mapper : mappers) {
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

      for (auto& mapper : mappers) {
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

      for (auto& mapper : mappers) {
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

      for (auto& mapper : mappers) {
        mapper->SetInputConnection(transform_filter->GetOutputPort());
      }

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
        vtkNew<vtkOpenGLTexture> texture;
        texture->SetInputConnection(texture_reader->GetOutputPort());
        texture->InterpolateOn();

        // TODO(kunimatsu-tri) Use proper values from material file for
        // ambient, diffuse and specular.
        // The current setting is hardcoded as to be (1, 1, 0) respectively.
        actors[ImageType::kColor]->GetProperty()->SetAmbient(1.);
        actors[ImageType::kColor]->SetTexture(texture.Get());
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
    auto& color_actor = actors[ImageType::kColor];
    if (color_actor->GetProperty()->GetNumberOfTextures() == 0) {
      const auto color = visual.getMaterial();
      color_actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    }

    auto& label_actor = actors[ImageType::kLabel];
    // This is to disable shadows and to get an object painted with a single
    // color.
    label_actor->GetProperty()->LightingOff();
    const auto& color = parent_->color_palette().get_normalized_color(body_id);
    label_actor->GetProperty()->SetColor(color.r, color.g, color.b);

    vtkSmartPointer<vtkTransform> vtk_transform =
        ConvertToVtkTransform(visual.getWorldTransform());
    auto& actor_collections = id_object_maps_[body_id];
    for (size_t i = 0; i < actors.size(); ++i) {
      actors[i]->SetMapper(mappers[i].GetPointer());
      actors[i]->SetUserTransform(vtk_transform);
      pipelines_[i]->renderer->AddActor(actors[i].GetPointer());
      actor_collections[i].push_back(actors[i].GetPointer());
    }

    return optional<VisualIndex>(VisualIndex(static_cast<int>(
        id_object_maps_[body_id][ImageType::kColor].size() - 1)));
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

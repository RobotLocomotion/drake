#include "drake/geometry/dev/render/render_engine_vtk.h"

#include <limits>
#include <stdexcept>
#include <utility>

#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkOBJReader.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "drake/geometry/dev/render/shaders/depth_shaders.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/vtk_util.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {

using std::make_unique;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::InvalidDepth;
using systems::sensors::vtk_util::ConvertToVtkTransform;
using systems::sensors::vtk_util::CreateSquarePlane;
using systems::sensors::vtk_util::MakeVtkPointerArray;

namespace {

const int kNumMaxLabel = 256;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
// TODO(SeanCurtis-TRI): For depth cameras, the clipping planes should be a
// function of the reportable z_near and z_far values; if the clipping planes
// are *beyond* those values, we've wasted precision.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;

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

float CheckRangeAndConvertToMeters(float z_buffer_value, double z_near,
                                   double z_far) {
  const double kA = z_far - kClippingPlaneNear;
  // Initialize with the assumption that the buffer value is outside the range
  // [kClippingPlaneNear, kClippingPlaneFar]. If the buffer value is *not* 1,
  // then it lies inside the range.
  float z = std::numeric_limits<float>::quiet_NaN();
  if (z_buffer_value != 1.f) {
    z = static_cast<float>(z_buffer_value * kA + kClippingPlaneNear);
    if (z > z_far) {
      z = InvalidDepth::kTooFar;
    } else if (z < z_near) {
      z = InvalidDepth::kTooClose;
    }
  }
  return z;
}

// TODO(SeanCurtis-TRI): This should ultimately part of the public SceneGraph
// API.
enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

// TODO(SeanCurtis-TRI): Add X_PG pose to this data.
// A package of data required to register a visual geometry.
struct RegistrationData {
  const PerceptionProperties& properties;
  const Isometry3<double>& X_FG;
  // The file name if the shape being registered is a mesh.
  optional<std::string> mesh_filename;
};

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    throw std::logic_error("File has no extension.");
  }
  return filepath.substr(0, last_dot);
}

}  // namespace

namespace detail {

ShaderCallback::ShaderCallback() :
    z_near_(kClippingPlaneNear),
    z_far_(kClippingPlaneFar) {}

}  // namespace detail

vtkNew<detail::ShaderCallback> RenderEngineVtk::uniform_setting_callback_;

RenderEngineVtk::RenderEngineVtk()
    : color_palette_(kNumMaxLabel, RenderLabel::terrain_label(),
                     RenderLabel::empty_label()),
      pipelines_{{make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>()}} {
  InitializePipelines();
}

void RenderEngineVtk::UpdateViewpoint(const Eigen::Isometry3d& X_WC) const {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (const auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RenderEngineVtk::RenderColorImage(const CameraProperties& camera,
                                       ImageRgba8U* color_image_out,
                                       bool show_window) const {
  UpdateWindow(camera, show_window, pipelines_[ImageType::kColor].get(),
               "Color Image");
  PerformVTKUpdate(*pipelines_[ImageType::kColor]);

  // TODO(SeanCurtis-TRI): Determine if this copies memory (and find some way
  // around copying).
  pipelines_[ImageType::kColor]->exporter->Export(color_image_out->at(0, 0));
}

void RenderEngineVtk::RenderDepthImage(const DepthCameraProperties& camera,
                                       ImageDepth32F* depth_image_out) const {
  UpdateWindow(camera, pipelines_[ImageType::kDepth].get());
  PerformVTKUpdate(*pipelines_[ImageType::kDepth]);

  // TODO(SeanCurtis-TRI): This copies the image and *that's* a tragedy. It
  // would be much better to process the pixels directly. The solution is to
  // simply call exporter->GetPoitnerToData() and process the pixels myself.
  // See the implementation in vtkImageExport::Export() for details.
  ImageRgba8U image(camera.width, camera.height);
  pipelines_[ImageType::kDepth]->exporter->Export(image.at(0, 0));

  for (int v = 0; v < camera.height; ++v) {
    for (int u = 0; u < camera.width; ++u) {
      if (image.at(u, v)[0] == 255u &&
          image.at(u, v)[1] == 255u &&
          image.at(u, v)[2] == 255u) {
        depth_image_out->at(u, v)[0] = InvalidDepth::kTooFar;
      } else {
        // Decoding three channel color values to a float value. For the detail,
        // see depth_shaders.h.
        float shader_value = image.at(u, v)[0] + image.at(u, v)[1] / 255. +
                             image.at(u, v)[2] / (255. * 255.);

        // Dividing by 255 so that the range gets to be [0, 1].
        shader_value /= 255.f;
        // TODO(kunimatsu-tri) Calculate this in a vertex shader.
        depth_image_out->at(u, v)[0] =
            CheckRangeAndConvertToMeters(shader_value, camera.z_near,
                                         camera.z_far);
      }
    }
  }
}

void RenderEngineVtk::RenderLabelImage(const CameraProperties& camera,
                                       ImageLabel16I* label_image_out,
                                       bool show_window) const {
  // TODO(SeanCurtis-TRI): Rather than rendering human readable palette
  // colors, the labels should be encoded as colors *directly* and only
  // re-encoded into colors when visualization is required. In other words,
  // we should render directly to the computationally meaningful format and
  // leave the human filter as a post-processing affect.
  // TODO(sherm1) Should evaluate VTK cache entry.
  UpdateWindow(camera, show_window, pipelines_[ImageType::kLabel].get(),
               "Label Image");
  PerformVTKUpdate(*pipelines_[ImageType::kLabel]);

  // TODO(SeanCurtis-TRI): This copies the image and *that's* a tragedy. It
  // would be much better to process the pixels directly. The solution is to
  // simply call exporter->GetPointerToData() and process the pixels myself.
  // See the implementation in vtkImageExport::Export() for details.
  ImageRgba8U image(camera.width, camera.height);
  pipelines_[ImageType::kLabel]->exporter->Export(image.at(0, 0));

  ColorI color;
  for (int v = 0; v < camera.height; ++v) {
    for (int u = 0; u < camera.width; ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      label_image_out->at(u, v)[0] = color_palette_.LookUpId(color);
    }
  }
}

void RenderEngineVtk::ImplementGeometry(const Sphere& sphere, void* user_data) {
  vtkNew<vtkSphereSource> vtk_sphere;
  vtk_sphere->SetRadius(sphere.get_radius());
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_sphere->SetThetaResolution(50);
  vtk_sphere->SetPhiResolution(50);
  ImplementGeometry(vtk_sphere.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Cylinder& cylinder,
                                        void* user_data) {
  vtkNew<vtkCylinderSource> vtk_cylinder;
  vtk_cylinder->SetHeight(cylinder.get_length());
  vtk_cylinder->SetRadius(cylinder.get_radius());
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_cylinder->SetResolution(50);

  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  transform->RotateX(90);
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
  transform_filter->SetTransform(transform.GetPointer());
  transform_filter->Update();

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const HalfSpace&,
                                        void* user_data) {
  vtkSmartPointer<vtkPlaneSource> vtk_plane = CreateSquarePlane(kTerrainSize);

  ImplementGeometry(vtk_plane.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Box& box, void* user_data) {
  vtkNew<vtkCubeSource> cube;
  cube->SetXLength(box.width());
  cube->SetYLength(box.depth());
  cube->SetZLength(box.height());
  ImplementGeometry(cube.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Mesh& mesh, void* user_data) {
  ImplementObj(mesh.filename(), mesh.scale(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Convex& convex, void* user_data) {
  ImplementObj(convex.filename(), convex.scale(), user_data);
}

/** Returns the sky's color in an RGB image. */
const ColorI& RenderEngineVtk::get_sky_color() const {
  return color_palette_.get_sky_color();
}

/** Returns flat terrain's color in an RGB image. */
const ColorI& RenderEngineVtk::get_flat_terrain_color() const {
  return color_palette_.get_terrain_color();
}

optional<RenderIndex> RenderEngineVtk::DoRegisterVisual(
    const Shape& shape, const PerceptionProperties& properties,
    const Isometry3<double>& X_FG) {
  // Note: the user_data interface on reification requires a non-const pointer.
  RegistrationData data{properties, X_FG};
  shape.Reify(this, &data);
  return RenderIndex(static_cast<int>(actors_.size()) - 1);
}

void RenderEngineVtk::DoUpdateVisualPose(const Eigen::Isometry3d& X_WG,
                                         RenderIndex index) {
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(X_WG);
  // TODO(SeanCurtis-TRI): Provide the ability to specify specific actors; i.e.
  // only update the visual actor or only the label actor, etc.
  for (const auto& actor : actors_.at(index)) {
    actor->SetUserTransform(vtk_X_WG);
  }
}

optional<RenderIndex> RenderEngineVtk::DoRemoveGeometry(RenderIndex index) {
  DRAKE_DEMAND(index >= 0 && index < actors_.size());
  for (int i = 0; i < 3; ++i) {
    pipelines_[i]->renderer->RemoveActor(actors_[index][i]);
  }
  optional<RenderIndex> moved_index{};
  RenderIndex last_index{static_cast<int>(actors_.size()) - 1};
  if (index < last_index) {
    moved_index = last_index;
    std::swap(actors_[index], actors_[last_index]);
    actors_.pop_back();
  }
  return moved_index;
}

std::unique_ptr<RenderEngine> RenderEngineVtk::DoClone() const {
  return std::unique_ptr<RenderEngineVtk>(new RenderEngineVtk(*this));
}

RenderEngineVtk::RenderEngineVtk(const RenderEngineVtk& other)
    : RenderEngine(other),
      color_palette_(kNumMaxLabel, RenderLabel::terrain_label(),
                     RenderLabel::empty_label()),
      pipelines_{{make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>()}} {
  InitializePipelines();

  // Utility function for creating a cloned actor which *shares* the same
  // underlying polygonal data.
  auto clone_actor_array =
      [this](const std::array<vtkSmartPointer<vtkActor>, 3>& source_actors,
             std::array<vtkSmartPointer<vtkActor>, 3>* clone_actors_ptr) {
        DRAKE_DEMAND(clone_actors_ptr != nullptr);
        std::array<vtkSmartPointer<vtkActor>, 3>& clone_actors =
            *clone_actors_ptr;
        for (int i = 0; i < 3; ++i) {
          // NOTE: source *should* be const; but none of the getters on the
          // source are const-compatible.
          DRAKE_DEMAND(source_actors[i]);
          DRAKE_DEMAND(clone_actors[i]);
          vtkActor& source = *source_actors[i];
          vtkActor& clone = *clone_actors[i];

          if (source.GetTexture() == nullptr) {
            clone.GetProperty()->SetColor(source.GetProperty()->GetColor());
          } else {
            clone.SetTexture(source.GetTexture());
          }
          clone.SetUserTransform(source.GetUserTransform());

          // NOTE: The clone renderer and original renderer *share* polygon
          // data. If the meshes were *deformable* this would be invalid.
          // Furthermore, even if dynamic adding/removing of geometry were
          // valid, VTK's reference counting preserves the underlying geometry
          // in the copy that still references it.
          clone.SetMapper(source.GetMapper());
          clone.SetUserTransform(source.GetUserTransform());
          // This is necessary because *terrain* has its lighting turned off. To
          // blindly handle arbitrary actors being flagged as terrain, we need
          // to treat all actors this way.
          clone.GetProperty()->SetLighting(source.GetProperty()->GetLighting());

          pipelines_.at(i)->renderer.Get()->AddActor(&clone);
        }
      };

  for (size_t a = 0; a < other.actors_.size(); ++a) {
    std::array<vtkSmartPointer<vtkActor>, 3> actors{
        vtkSmartPointer<vtkActor>::New(), vtkSmartPointer<vtkActor>::New(),
        vtkSmartPointer<vtkActor>::New()};
    clone_actor_array(other.actors_.at(a), &actors);
    actors_.emplace_back(actors);
  }

  // Copy camera properties
  auto copy_cameras = [](auto src_renderer, auto dst_renderer) {
    dst_renderer->GetActiveCamera()->DeepCopy(src_renderer->GetActiveCamera());
  };
  for (int p = 0; p < 3; ++p) {
    copy_cameras(other.pipelines_.at(p)->renderer.Get(),
                 pipelines_.at(p)->renderer.Get());
  }
}

void RenderEngineVtk::InitializePipelines() {
  const ColorD sky_color =
      systems::sensors::ColorPalette<int>::Normalize(get_sky_color());
  const vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(Eigen::Isometry3d::Identity());

  // Generic configuration of pipelines.
  for (auto& pipeline : pipelines_) {
    // Multisampling disabled by design for label and depth. It's turned off for
    // color because of a bug which affects on-screen rendering with NVidia
    // drivers on Ubuntu 16.04. In certain very specific
    // cases (camera position, scene, triangle drawing order, normal
    // orientation), a plane surface has partial background pixels
    // bleeding through it, which changes the color of the center pixel.
    // TODO(fbudin69500) If lack of anti-aliasing in production code is
    // problematic, change this to only disable anti-aliasing in unit
    // tests. Alternatively, find other way to resolve the driver bug.
    pipeline->window->SetMultiSamples(0);

    pipeline->renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(90.0);  // Default to an arbitrary 90° field of view.
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_identity);

    pipeline->window->AddRenderer(pipeline->renderer.GetPointer());
    pipeline->filter->SetInput(pipeline->window.GetPointer());
    pipeline->filter->SetScale(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();
  }

  // Pipeline-specific tweaks.

  // Depth image background color is white -- the representation of the maximum
  // distance (e.g., infinity).
  pipelines_[ImageType::kDepth]->renderer->SetBackground(1., 1., 1.);

  pipelines_[ImageType::kColor]->renderer->SetUseDepthPeeling(1);
  pipelines_[ImageType::kColor]->renderer->UseFXAAOn();
}

void RenderEngineVtk::ImplementObj(const std::string& file_name, double scale,
                                   void* user_data) {
  static_cast<RegistrationData*>(user_data)->mesh_filename = file_name;
  vtkNew<vtkOBJReader> mesh_reader;
  mesh_reader->SetFileName(file_name.c_str());
  mesh_reader->Update();

  vtkNew<vtkTransform> transform;
  // TODO(SeanCurtis-TRI): Should I be allowing only isotropic scale.
  // TODO(SeanCurtis-TRI): Only add the transform filter if scale is not all 1.
  transform->Scale(scale, scale, scale);
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(mesh_reader->GetOutputPort());
  transform_filter->SetTransform(transform.GetPointer());
  transform_filter->Update();

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(vtkPolyDataAlgorithm* source,
                                        void* user_data) {
  DRAKE_DEMAND(user_data != nullptr);

  std::array<vtkSmartPointer<vtkActor>, 3> actors{
      vtkSmartPointer<vtkActor>::New(), vtkSmartPointer<vtkActor>::New(),
      vtkSmartPointer<vtkActor>::New()};
  std::array<vtkNew<vtkOpenGLPolyDataMapper>, 3> mappers;

  // Sets vertex and fragment shaders only to the depth mapper.
  mappers[ImageType::kDepth]->SetVertexShaderCode(shaders::kDepthVS);
  mappers[ImageType::kDepth]->SetFragmentShaderCode(shaders::kDepthFS);
  mappers[ImageType::kDepth]->AddObserver(
      vtkCommand::UpdateShaderEvent, uniform_setting_callback_.Get());

  for (auto& mapper : mappers) {
    mapper->SetInputConnection(source->GetOutputPort());
  }

  const RegistrationData& data =
      *reinterpret_cast<RegistrationData*>(user_data);

  auto& label_actor = actors[ImageType::kLabel];
  // This is to disable shadows and to get an object painted with a single
  // color.
  label_actor->GetProperty()->LightingOff();
  const RenderLabel label = data.properties.GetPropertyOrDefault(
      "label", "id", RenderLabel::terrain_label());
  const auto color = color_palette_.get_normalized_color(label);
  label_actor->GetProperty()->SetColor(color.r, color.g, color.b);

  // Color actor.
  auto& color_actor = actors[ImageType::kColor];

  const std::string& diffuse_map_name =
      data.properties.GetPropertyOrDefault<std::string>("phong", "diffuse_map",
                                                        "");
  std::string texture_name;
  std::ifstream file_exist(diffuse_map_name);
  if (file_exist) {
    texture_name = diffuse_map_name;
  } else if (diffuse_map_name.empty() && data.mesh_filename) {
    // This is the hack to search for mesh.png as a possible texture.
    const std::string
        alt_texture_name(RemoveFileExtension(*data.mesh_filename) +
        ".png");
    std::ifstream alt_file_exist(alt_texture_name);
    if (alt_file_exist) texture_name = alt_texture_name;
  }
  if (!texture_name.empty()) {
    vtkNew<vtkPNGReader> texture_reader;
    texture_reader->SetFileName(texture_name.c_str());
    texture_reader->Update();
    vtkNew<vtkOpenGLTexture> texture;
    texture->SetInputConnection(texture_reader->GetOutputPort());
    texture->InterpolateOn();
    color_actor->SetTexture(texture.Get());
  } else {
      // TODO(SeanCurtis-TRI): Make this default color part of the public API so
      // it can be configured.
      // Default is an obnoxious orange to help flag un-defined values.
      const Eigen::Vector4d default_diffuse(0.9, 0.45, 0.1, 1.0);
      const Eigen::Vector4d& diffuse =
          data.properties.GetPropertyOrDefault("phong", "diffuse",
                                               default_diffuse);
      color_actor->GetProperty()->SetColor(diffuse(0), diffuse(1), diffuse(2));
  }
  // TODO(SeanCurtis-TRI): It is not clear why this isn't illuminated. Document
  // this behavior when I change "terrain" label to "don't care" label.
  if (label.is_terrain()) {
    color_actor->GetProperty()->LightingOff();
  }

  // TODO(SeanCurtis-TRI): For anchored geometry, I need to know its pose: X_PG,
  // where P = W. For all other geometries, the value is irrelevant (and will
  // be supplanted in the pose update).
  vtkSmartPointer<vtkTransform> vtk_X_PG = ConvertToVtkTransform(data.X_FG);
  for (size_t i = 0; i < 3; ++i) {
    actors[i]->SetMapper(mappers[i].GetPointer());
    actors[i]->SetUserTransform(vtk_X_PG);
    pipelines_[i]->renderer->AddActor(actors[i].GetPointer());
  }
  actors_.emplace_back(actors);
}

void RenderEngineVtk::PerformVTKUpdate(const RenderingPipeline& p) {
  p.window->Render();
  p.filter->Modified();
  p.filter->Update();
}

void RenderEngineVtk::UpdateWindow(const CameraProperties& camera,
                                   bool show_window,
                                   const RenderingPipeline* p,
                                   const char* name) const {
  // NOTE: This is a horrible hack for modifying what otherwise looks like
  // const entities.
  p->window->SetSize(camera.width, camera.height);
  p->window->SetOffScreenRendering(!show_window);
  if (show_window) p->window->SetWindowName(name);
  p->renderer->GetActiveCamera()->SetViewAngle(camera.fov_y * 180 / M_PI);
}

void RenderEngineVtk::UpdateWindow(const DepthCameraProperties& camera,
                                   const RenderingPipeline* p) const {
  p->renderer->GetActiveCamera()->SetClippingRange(kClippingPlaneNear,
                                                   camera.z_far);

  // TODO(SeanCurtis-TRI): Better explain this particular configuration; what
  // does kClippingPlaneNear give us?
  // Setting kClippingPlaneNear instead of z_near_ so that we can distinguish
  // kTooClose from out of range.
  uniform_setting_callback_->set_z_near(kClippingPlaneNear);
  uniform_setting_callback_->set_z_far(static_cast<float>(camera.z_far));
  UpdateWindow(camera, false, p, "Depth Image");
}

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake

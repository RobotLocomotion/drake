#include "drake/geometry/render/render_engine_vtk.h"

#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include <vtkCamera.h>
#include <vtkCylinderSource.h>
#include <vtkOBJReader.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkTexturedSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/render/render_engine_vtk_base.h"
#include "drake/geometry/render/shaders/depth_shaders.h"
#include "drake/geometry/render/vtk_util.h"
#include "drake/systems/sensors/color_palette.h"

namespace drake {
namespace geometry {
namespace render {

using Eigen::Vector2d;
using Eigen::Vector4d;
using math::RigidTransformd;
using std::make_unique;
using systems::sensors::CameraInfo;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;
using vtk_util::ConvertToVtkTransform;
using vtk_util::CreateSquarePlane;
using vtk_util::MakeVtkPointerArray;

namespace {

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
  // This assumes the values have already been encoded to the range [0, 1]
  // inside the image itself. This includes sentinel values:
  //   - The value 1 indicates the depth is too far away.
  //   - The value 0 indicates the depth is too near.
  if (z_buffer_value == 0) return ImageTraits<PixelType::kDepth32F>::kTooClose;
  if (z_buffer_value == 1) return ImageTraits<PixelType::kDepth32F>::kTooFar;
  return static_cast<float>(z_buffer_value * (z_far - z_near) + z_near);
}

enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

// TODO(SeanCurtis-TRI): Add X_PG pose to this data.
// A package of data required to register a visual geometry.
struct RegistrationData {
  const PerceptionProperties& properties;
  const RigidTransformd& X_WG;
  const GeometryId id;
  // The file name if the shape being registered is a mesh.
  std::optional<std::string> mesh_filename;
};

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    throw std::logic_error("File has no extension.");
  }
  return filepath.substr(0, last_dot);
}

}  // namespace

namespace internal {

ShaderCallback::ShaderCallback() :
    // These values are arbitrary "reasonable" values, but we expect them to
    // *both* be overwritten upon every usage.
    z_near_(0.01),
    z_far_(100.0) {}

}  // namespace internal

vtkNew<internal::ShaderCallback> RenderEngineVtk::uniform_setting_callback_;

RenderEngineVtk::RenderEngineVtk(const RenderEngineVtkParams& parameters)
    : RenderEngine(parameters.default_label ? *parameters.default_label
                                            : RenderLabel::kUnspecified),
      pipelines_{{make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>()}} {
  if (parameters.default_diffuse) {
    default_diffuse_ = *parameters.default_diffuse;
  }

  const auto& c = parameters.default_clear_color;
  default_clear_color_ = ColorD{c(0), c(1), c(2)};

  InitializePipelines();
}

void RenderEngineVtk::UpdateViewpoint(const RigidTransformd& X_WC) {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (const auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RenderEngineVtk::ImplementGeometry(const Sphere& sphere, void* user_data) {
  vtkNew<vtkTexturedSphereSource> vtk_sphere;
  SetSphereOptions(vtk_sphere.GetPointer(), sphere.radius());
  ImplementGeometry(vtk_sphere.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Cylinder& cylinder,
                                        void* user_data) {
  vtkNew<vtkCylinderSource> vtk_cylinder;
  SetCylinderOptions(vtk_cylinder, cylinder.length(), cylinder.radius());

  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  TransformToDrakeCylinder(transform, transform_filter, vtk_cylinder);

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const HalfSpace&,
                                        void* user_data) {
  vtkSmartPointer<vtkPlaneSource> vtk_plane = CreateSquarePlane(kTerrainSize);

  ImplementGeometry(vtk_plane.GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Box& box, void* user_data) {
  const RegistrationData* data = static_cast<RegistrationData*>(user_data);
  ImplementGeometry(CreateVtkBox(box, data->properties).GetPointer(),
                    user_data);
}

void RenderEngineVtk::ImplementGeometry(const Capsule& capsule,
                                        void* user_data) {
  ImplementGeometry(CreateVtkCapsule(capsule).GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Ellipsoid& ellipsoid,
                                        void* user_data) {
  ImplementGeometry(CreateVtkEllipsoid(ellipsoid).GetPointer(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Mesh& mesh, void* user_data) {
  ImplementObj(mesh.filename(), mesh.scale(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Convex& convex, void* user_data) {
  ImplementObj(convex.filename(), convex.scale(), user_data);
}

bool RenderEngineVtk::DoRegisterVisual(
    GeometryId id, const Shape& shape, const PerceptionProperties& properties,
    const RigidTransformd& X_WG) {
  // Note: the user_data interface on reification requires a non-const pointer.
  RegistrationData data{properties, X_WG, id};
  shape.Reify(this, &data);
  return true;
}

void RenderEngineVtk::DoUpdateVisualPose(GeometryId id,
                                         const RigidTransformd& X_WG) {
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(X_WG);
  // TODO(SeanCurtis-TRI): Perhaps provide the ability to specify actors for
  //  specific pipelines; i.e. only update the color actor or only the label
  //  actor, etc.
  for (const auto& actor : actors_.at(id)) {
    actor->SetUserTransform(vtk_X_WG);
  }
}

bool RenderEngineVtk::DoRemoveGeometry(GeometryId id) {
  auto iter = actors_.find(id);

  if (iter != actors_.end()) {
    std::array<vtkSmartPointer<vtkActor>, 3>& pipe_actors = iter->second;
    for (int i = 0; i < kNumPipelines; ++i) {
      // If the label actor hasn't been added to its renderer, this is a no-op.
      pipelines_[i]->renderer->RemoveActor(pipe_actors[i]);
    }
    actors_.erase(iter);
    return true;
  }

  return false;
}

std::unique_ptr<RenderEngine> RenderEngineVtk::DoClone() const {
  return std::unique_ptr<RenderEngineVtk>(new RenderEngineVtk(*this));
}

void RenderEngineVtk::DoRenderColorImage(
    const ColorRenderCamera& camera,
    ImageRgba8U* color_image_out) const {
  UpdateWindow(camera.core(), camera.show_window(),
               pipelines_[ImageType::kColor].get(), "Color Image");
  PerformVtkUpdate(*pipelines_[ImageType::kColor]);

  // TODO(SeanCurtis-TRI): Determine if this copies memory (and find some way
  // around copying).
  pipelines_[ImageType::kColor]->exporter->Export(color_image_out->at(0, 0));
}

void RenderEngineVtk::DoRenderDepthImage(
    const DepthRenderCamera& camera,
      ImageDepth32F* depth_image_out) const {
  UpdateWindow(camera, pipelines_[ImageType::kDepth].get());
  PerformVtkUpdate(*pipelines_[ImageType::kDepth]);

  const CameraInfo& intrinsics = camera.core().intrinsics();
  ImageRgba8U image(intrinsics.width(), intrinsics.height());
  // TODO(SeanCurtis-TRI): We're doing multiple passes on the pixel data. This
  // does one pass by copying the filter to the given image. We then do a second
  // pass where we re-encode the values. It would be much better to process the
  // pixels in a single pass.  The solution is to simply call
  // exporter->GetPointerToData() and process the pixels as they are read.
  // See the implementation in vtkImageExport::Export() for details.
  pipelines_[ImageType::kDepth]->exporter->Export(image.at(0, 0));

  const double min_depth = camera.depth_range().min_depth();
  const double max_depth = camera.depth_range().max_depth();
  for (int v = 0; v < intrinsics.height(); ++v) {
    for (int u = 0; u < intrinsics.width(); ++u) {
      if (image.at(u, v)[0] == 255u && image.at(u, v)[1] == 255u &&
          image.at(u, v)[2] == 255u) {
        depth_image_out->at(u, v)[0] =
            ImageTraits<PixelType::kDepth32F>::kTooFar;
      } else {
        // Decoding three channel color values to a float value. For the detail,
        // see depth_shaders.h.
        float shader_value = image.at(u, v)[0] + image.at(u, v)[1] / 255. +
                             image.at(u, v)[2] / (255. * 255.);

        // Dividing by 255 so that the range gets to be [0, 1].
        shader_value /= 255.f;
        // TODO(kunimatsu-tri) Calculate this in a vertex shader.
        depth_image_out->at(u, v)[0] = CheckRangeAndConvertToMeters(
            shader_value, min_depth, max_depth);
      }
    }
  }
}

void RenderEngineVtk::DoRenderLabelImage(
    const ColorRenderCamera& camera,
    ImageLabel16I* label_image_out) const {
  UpdateWindow(camera.core(), camera.show_window(),
               pipelines_[ImageType::kLabel].get(), "Label Image");
  PerformVtkUpdate(*pipelines_[ImageType::kLabel]);

  // TODO(SeanCurtis-TRI): This copies the image and *that's* a tragedy. It
  // would be much better to process the pixels directly. The solution is to
  // simply call exporter->GetPointerToData() and process the pixels myself.
  // See the implementation in vtkImageExport::Export() for details.
  const CameraInfo& intrinsics = camera.core().intrinsics();
  ImageRgba8U image(intrinsics.width(), intrinsics.height());
  pipelines_[ImageType::kLabel]->exporter->Export(image.at(0, 0));

  ColorI color;
  for (int v = 0; v < intrinsics.height(); ++v) {
    for (int u = 0; u < intrinsics.width(); ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      label_image_out->at(u, v)[0] = RenderEngine::LabelFromColor(color);
    }
  }
}

RenderEngineVtk::RenderEngineVtk(const RenderEngineVtk& other)
    : RenderEngine(other),
      pipelines_{{make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>()}},
      default_diffuse_{other.default_diffuse_},
      default_clear_color_{other.default_clear_color_} {
  InitializePipelines();

  // Utility function for creating a cloned actor which *shares* the same
  // underlying polygonal data.
  auto clone_actor_array = [this](
      const std::array<vtkSmartPointer<vtkActor>, kNumPipelines>& source_actors,
      std::array<vtkSmartPointer<vtkActor>, kNumPipelines>* clone_actors_ptr) {
    DRAKE_DEMAND(clone_actors_ptr != nullptr);
    std::array<vtkSmartPointer<vtkActor>, kNumPipelines>& clone_actors =
        *clone_actors_ptr;
    for (int i = 0; i < kNumPipelines; ++i) {
      // NOTE: source *should* be const; but none of the getters on the source
      // are const-compatible.
      DRAKE_DEMAND(source_actors[i]);
      DRAKE_DEMAND(clone_actors[i]);
      vtkActor& source = *source_actors[i];
      vtkActor& clone = *clone_actors[i];

      // NOTE: The clone renderer and original renderer *share* polygon data
      // (via the shared mapper) and all textures. If the meshes or textures get
      // modified _in place_ in a clone, the change would be visible to all
      // copies of the renderer. If that proves to be problematic we'll have to
      // make the copy "deeper" as appropriate.
      if (source.GetTexture() == nullptr) {
        clone.GetProperty()->SetColor(source.GetProperty()->GetColor());
        clone.GetProperty()->SetOpacity(source.GetProperty()->GetOpacity());
      } else {
        clone.SetTexture(source.GetTexture());
      }
      const auto& source_textures = source.GetProperty()->GetAllTextures();
      for (auto& [name, texture] : source_textures) {
        clone.GetProperty()->SetTexture(name.c_str(), texture);
      }

      clone.SetMapper(source.GetMapper());
      clone.SetUserTransform(source.GetUserTransform());
      // This is necessary because *terrain* has its lighting turned off. To
      // blindly handle arbitrary actors being flagged as terrain, we need
      // to treat all actors this way.
      clone.GetProperty()->SetLighting(source.GetProperty()->GetLighting());

      pipelines_.at(i)->renderer.Get()->AddActor(&clone);
    }
  };

  for (const auto& other_id_actor_pair : other.actors_) {
    std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
        vtkSmartPointer<vtkActor>::New(), vtkSmartPointer<vtkActor>::New(),
        vtkSmartPointer<vtkActor>::New()};
    clone_actor_array(other_id_actor_pair.second, &actors);
    const GeometryId id = other_id_actor_pair.first;
    actors_.insert({id, move(actors)});
  }

  // Copy camera properties
  auto copy_cameras = [](auto src_renderer, auto dst_renderer) {
    dst_renderer->GetActiveCamera()->DeepCopy(src_renderer->GetActiveCamera());
  };
  for (int p = 0; p < kNumPipelines; ++p) {
    copy_cameras(other.pipelines_.at(p)->renderer.Get(),
                 pipelines_.at(p)->renderer.Get());
  }
}

void RenderEngineVtk::InitializePipelines() {
  const vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(RigidTransformd::Identity());

  // TODO(SeanCurtis-TRI): Things like configuring lights should *not* be part
  //  of initializing the pipelines. When we support light declaration, this
  //  will get moved out.
  light_->SetLightTypeToCameraLight();
  light_->SetConeAngle(45.0);
  light_->SetAttenuationValues(1.0, 0.0, 0.0);
  light_->SetIntensity(1);
  light_->SetTransformMatrix(vtk_identity->GetMatrix());

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

    auto* camera = pipeline->renderer->GetActiveCamera();
    camera->UseExplicitProjectionTransformMatrixOn();
    vtkNew<vtkMatrix4x4> projection;
    projection->Zero();
    camera->SetExplicitProjectionTransformMatrix(projection.Get());

    SetModelTransformMatrixToVtkCamera(camera, vtk_identity);

    pipeline->window->AddRenderer(pipeline->renderer.GetPointer());
    pipeline->filter->SetInput(pipeline->window.GetPointer());
    pipeline->filter->SetScale(1);
    // TODO(SeanCurtis-TRI): In (at least) VTK 8.2, there is an error with
    // using an explicitly set projection matrix and this property being set to
    // true. Essentially, the "re render" copies the camera, and the camera
    // copy fails to include explicit projection matrix bits. See:
    // https://gitlab.kitware.com/vtk/vtk/-/issues/17520#note_776238
    pipeline->filter->SetShouldRerender(false);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();

    pipeline->renderer->AddLight(light_);
  }

  // Pipeline-specific tweaks.

  // Depth image background color is white -- the representation of the maximum
  // distance (e.g., infinity).
  pipelines_[ImageType::kDepth]->renderer->SetBackground(1., 1., 1.);

  const ColorD empty_color =
      RenderEngine::GetColorDFromLabel(RenderLabel::kEmpty);
  pipelines_[ImageType::kLabel]->renderer->SetBackground(
      empty_color.r, empty_color.g, empty_color.b);

  pipelines_[ImageType::kColor]->renderer->SetUseDepthPeeling(1);
  pipelines_[ImageType::kColor]->renderer->UseFXAAOn();
  pipelines_[ImageType::kColor]->renderer->SetBackground(
      default_clear_color_.r, default_clear_color_.g, default_clear_color_.b);
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

  std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
      vtkSmartPointer<vtkActor>::New(), vtkSmartPointer<vtkActor>::New(),
      vtkSmartPointer<vtkActor>::New()};
  // Note: the mappers ultimately get referenced by the actors, so they do _not_
  // get destroyed when this array goes out of scope.
  std::array<vtkNew<vtkOpenGLPolyDataMapper>, kNumPipelines> mappers;

  // Sets vertex and fragment shaders only to the depth mapper.
  mappers[ImageType::kDepth]->SetVertexShaderCode(shaders::kDepthVS);
  mappers[ImageType::kDepth]->SetFragmentShaderCode(shaders::kDepthFS);
  mappers[ImageType::kDepth]->AddObserver(
      vtkCommand::UpdateShaderEvent, uniform_setting_callback_.Get());

  for (auto& mapper : mappers) {
    mapper->SetInputConnection(source->GetOutputPort());
  }

  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);

  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(data.X_WG);

  // Adds the actor into the specified pipeline.
  auto connect_actor = [this, &actors, &mappers,
                        &vtk_X_WG](ImageType image_type) {
    actors[image_type]->SetMapper(mappers[image_type].Get());
    actors[image_type]->SetUserTransform(vtk_X_WG);
    pipelines_[image_type]->renderer->AddActor(actors[image_type].Get());
  };

  // Label actor.
  const RenderLabel label = GetRenderLabelOrThrow(data.properties);
  if (label != RenderLabel::kDoNotRender) {
    // NOTE: We only configure the label actor if it doesn't have to "do not
    // render" label applied. We *have* created an actor and connected it to
    // a mapper; but otherwise, we leave it disconnected.
    auto& label_actor = actors[ImageType::kLabel];
    // This is to disable shadows and to get an object painted with a single
    // color.
    label_actor->GetProperty()->LightingOff();
    const auto color = RenderEngine::GetColorDFromLabel(label);
    label_actor->GetProperty()->SetColor(color.r, color.g, color.b);
    connect_actor(ImageType::kLabel);
  }

  // Color actor.
  auto& color_actor = actors[ImageType::kColor];
  const std::string& diffuse_map_name =
      data.properties.GetPropertyOrDefault<std::string>("phong", "diffuse_map",
                                                        "");
  // Legacy support for *implied* texture maps. If we have mesh.obj, we look for
  // mesh.png (unless one has been specifically called out in the properties).
  // TODO(SeanCurtis-TRI): Remove this legacy texture when objects and materials
  //  are coherently specified by SDF/URDF/obj/mtl, etc.
  std::string texture_name;
  std::ifstream file_exist(diffuse_map_name);
  if (file_exist) {
    texture_name = diffuse_map_name;
  } else {
    if (!diffuse_map_name.empty()) {
      log()->warn("Requested diffuse map could not be found: {}",
                  diffuse_map_name);
    }
    if (diffuse_map_name.empty() && data.mesh_filename) {
      // This is the hack to search for mesh.png as a possible texture.
      const std::string alt_texture_name(
          RemoveFileExtension(*data.mesh_filename) + ".png");
      std::ifstream alt_file_exist(alt_texture_name);
      if (alt_file_exist) texture_name = alt_texture_name;
    }
  }
  if (!texture_name.empty()) {
    const Vector2d& uv_scale = data.properties.GetPropertyOrDefault(
        "phong", "diffuse_scale", Vector2d{1, 1});
    vtkNew<vtkPNGReader> texture_reader;
    texture_reader->SetFileName(texture_name.c_str());
    texture_reader->Update();
    vtkNew<vtkOpenGLTexture> texture;
    texture->SetInputConnection(texture_reader->GetOutputPort());
    const bool need_repeat = uv_scale[0] > 1 || uv_scale[1] > 1;
    texture->SetRepeat(need_repeat);
    texture->InterpolateOn();
    color_actor->SetTexture(texture.Get());
  } else {
    const Vector4d& diffuse =
        data.properties.GetPropertyOrDefault("phong", "diffuse",
                                             default_diffuse_);
    color_actor->GetProperty()->SetColor(diffuse(0), diffuse(1), diffuse(2));
    color_actor->GetProperty()->SetOpacity(diffuse(3));
  }
  // TODO(SeanCurtis-TRI): Determine if this precludes modulating the texture
  //  with arbitrary rgba values (e.g., tinting red or making everything
  //  slightly transparent). In other words, should opacity be set regardless
  //  of whether a texture exists?

  connect_actor(ImageType::kColor);

  // Depth actor; always gets wired in with no additional work.
  connect_actor(ImageType::kDepth);

  // Take ownership of the actors.
  actors_.insert({data.id, std::move(actors)});
}

void RenderEngineVtk::SetDefaultLightPosition(const Vector3<double>& X_DL) {
  light_->SetPosition(X_DL[0], X_DL[1], X_DL[2]);
}

void RenderEngineVtk::PerformVtkUpdate(const RenderingPipeline& p) {
  p.window->Render();
  p.filter->Modified();
  p.filter->Update();
}

void RenderEngineVtk::UpdateWindow(const RenderCameraCore& camera,
                                   bool show_window, const RenderingPipeline* p,
                                   const char* name) const {
  // NOTE: Although declared const, this method modifies VTK entities. The
  // conflict between ostensibly const operations and invocation of black-box
  // entities that need state mutated should be more formally handled.

  const CameraInfo& intrinsics = camera.intrinsics();
  p->window->SetSize(intrinsics.width(), intrinsics.height());
  p->window->SetOffScreenRendering(!show_window);
  if (show_window) p->window->SetWindowName(name);

  vtkCamera* vtk_camera = p->renderer->GetActiveCamera();
  DRAKE_DEMAND(vtk_camera->GetUseExplicitProjectionTransformMatrix());
  vtkMatrix4x4* proj_mat = vtk_camera->GetExplicitProjectionTransformMatrix();
  DRAKE_DEMAND(proj_mat != nullptr);
  const Eigen::Matrix4f T_DC = camera.CalcProjectionMatrix().cast<float>();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      proj_mat->SetElement(i, j, T_DC(i, j));
    }
  }
  vtk_camera->Modified();
}

void RenderEngineVtk::UpdateWindow(const DepthRenderCamera& camera,
                                   const RenderingPipeline* p) const {
  uniform_setting_callback_->set_z_near(
      static_cast<float>(camera.depth_range().min_depth()));
  uniform_setting_callback_->set_z_far(
      static_cast<float>(camera.depth_range().max_depth()));
  // Never show window for depth camera; it is a meaningless operation as the
  // raw depth rasterization is not human consummable.
  UpdateWindow(camera.core(), false, p, "");
}

}  // namespace render
}  // namespace geometry
}  // namespace drake

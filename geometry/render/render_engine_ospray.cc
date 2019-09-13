#include "drake/geometry/render/render_engine_ospray.h"

#include <limits>
#include <stdexcept>
#include <utility>

#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkOBJReader.h>
#include <vtkOSPRayLightNode.h>
#include <vtkOSPRayMaterialLibrary.h>
#include <vtkOSPRayRendererNode.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/vtk_util.h"

namespace drake {
namespace geometry {
namespace render {

using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using std::make_unique;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::vtk_util::ConvertToVtkTransform;
using systems::sensors::vtk_util::CreateSquarePlane;

namespace {

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  // Sets z-forward.
  camera->SetFocalPoint(0., 0., 1.);
  // Sets y-down. For the detail, please refer to CameraInfo's document.
  camera->SetViewUp(0., -1, 0.);
  camera->ApplyTransform(X_WC);
}

// Note: the kLabel and kDepth values are not currently used. They are left in
// place for when this renderer can produce depth and label images.
enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

// TODO(SeanCurtis-TRI): Add X_PG pose to this data.
// A package of data required to register a visual geometry.
struct RegistrationData {
  const PerceptionProperties& properties;
  const RigidTransformd& X_FG;
  const GeometryId id;
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

RenderEngineOspray::RenderEngineOspray(const RenderEngineOsprayParams& params)
    : RenderEngine(RenderLabel::kUnspecified),
      pipelines_{{make_unique<RenderingPipeline>()}},
      render_mode_(params.mode) {
  if (params.default_diffuse) {
    default_diffuse_ = *params.default_diffuse;
  }

  if (params.background_color) {
    const Vector3d& c = *params.background_color;
    background_color_ = ColorD{c(0), c(1), c(2)};
  }

  InitializePipelines(params.samples_per_pixel);
}

void RenderEngineOspray::UpdateViewpoint(const RigidTransformd& X_WC) {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (const auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RenderEngineOspray::RenderColorImage(const CameraProperties& camera,
                                          bool show_window,
                                          ImageRgba8U* color_image_out) const {
  UpdateWindow(camera, show_window, pipelines_[ImageType::kColor].get(),
               "Color Image");
  PerformVtkUpdate(*pipelines_[ImageType::kColor]);

  // TODO(SeanCurtis-TRI): Determine if this copies memory (and find some way
  // around copying).
  auto& exporter = *pipelines_[ImageType::kColor]->exporter;
  DRAKE_DEMAND(exporter.GetDataNumberOfScalarComponents() == 4);
  exporter.Export(color_image_out->at(0, 0));

  // In path tracing, the background contributes light energy to the rendering.
  // However, the raytracer and gl renderers don't do this, it's simply a
  // backdrop -- a color for pixels that weren't otherwise covered by geometry.
  // We simulate that effect by blending the final pixels with the background
  // color.
  // TODO(SeanCurtis-TRI): Make this configurable; i.e., if we provide an hdmi
  //  environment map, this shouldn't happen at all.
  if (render_mode_ == OsprayMode::kPathTracer) {
    using ChannelType = ImageRgba8U::Traits::ChannelType;
    // Note: the cast truncates, by adding 0.5, it becomes rounding.
    systems::sensors::ColorI rgb{
        static_cast<ChannelType>(background_color_.r * 255 + 0.5),
        static_cast<ChannelType>(background_color_.g * 255 + 0.5),
        static_cast<ChannelType>(background_color_.b * 255 + 0.5)};

    auto blend = [](ImageRgba8U::Traits::ChannelType channel,
                    ImageRgba8U::Traits::ChannelType bg, double alpha) {
      double combo = channel * alpha + bg * (1 - alpha);
      return static_cast<ImageRgba8U::Traits::ChannelType>(combo);
    };

    for (int r = 0; r < color_image_out->height(); ++r) {
      for (int c = 0; c < color_image_out->width(); ++c) {
        ChannelType* pixel = color_image_out->at(c, r);
        double alpha = pixel[3] / 255.0;
        if (alpha < 1.0) {
          pixel[0] = blend(pixel[0], rgb.r, alpha);
          pixel[1] = blend(pixel[1], rgb.g, alpha);
          pixel[2] = blend(pixel[2], rgb.b, alpha);
        }
      }
    }
  }
}

void RenderEngineOspray::RenderDepthImage(const DepthCameraProperties&,
                                          ImageDepth32F*) const {
  throw std::runtime_error("RenderEngineOspray does not support depth images");
}

void RenderEngineOspray::RenderLabelImage(const CameraProperties&, bool,
                                       ImageLabel16I*) const {
  throw std::runtime_error("RenderEngineOspray does not support label images");
}

void RenderEngineOspray::ImplementGeometry(const Sphere& sphere,
                                           void* user_data) {
  // TODO(SeanCurtis-TRI): OSPRay supports a primitive sphere; find some way to
  //  exercise *that* instead of needlessly tesselating.
  vtkNew<vtkSphereSource> vtk_sphere;
  vtk_sphere->SetRadius(sphere.get_radius());
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_sphere->SetThetaResolution(50);
  vtk_sphere->SetPhiResolution(50);
  ImplementGeometry(vtk_sphere.GetPointer(), user_data);
}

void RenderEngineOspray::ImplementGeometry(const Cylinder& cylinder,
                                           void* user_data) {
  // TODO(SeanCurtis-TRI): OSPRay supports a primitive cylinder; find some way
  //  to exercise *that* instead of needlessly tesselating.
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

void RenderEngineOspray::ImplementGeometry(const HalfSpace&, void* user_data) {
  // TODO(SeanCurtis-TRI): Allow this to be configured upon construction.
  const double kTerrainSize = 100.;
  vtkSmartPointer<vtkPlaneSource> vtk_plane = CreateSquarePlane(kTerrainSize);

  ImplementGeometry(vtk_plane.GetPointer(), user_data);
}

void RenderEngineOspray::ImplementGeometry(const Box& box, void* user_data) {
  vtkNew<vtkCubeSource> cube;
  cube->SetXLength(box.width());
  cube->SetYLength(box.depth());
  cube->SetZLength(box.height());
  ImplementGeometry(cube.GetPointer(), user_data);
}

void RenderEngineOspray::ImplementGeometry(const Mesh& mesh, void* user_data) {
  ImplementObj(mesh.filename(), mesh.scale(), user_data);
}

void RenderEngineOspray::ImplementGeometry(const Convex& convex,
                                           void* user_data) {
  ImplementObj(convex.filename(), convex.scale(), user_data);
}

bool RenderEngineOspray::DoRegisterVisual(
    GeometryId id, const Shape& shape, const PerceptionProperties& properties,
    const RigidTransformd& X_FG) {
  // Note: the user_data interface on reification requires a non-const pointer.
  RegistrationData data{properties, X_FG, id};
  shape.Reify(this, &data);
  return true;
}

RenderEngineOsprayParams RenderEngineOspray::get_params() const {
  const int samples_per_pixel = vtkOSPRayRendererNode::GetSamplesPerPixel(
      pipelines_[ImageType::kColor]->renderer);
  return {
      render_mode_, default_diffuse_,
      Vector3d{background_color_.r, background_color_.g, background_color_.b},
      samples_per_pixel};
}

void RenderEngineOspray::DoUpdateVisualPose(GeometryId id,
                                            const RigidTransformd& X_WG) {
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(X_WG);
  // TODO(SeanCurtis-TRI): Perhaps provide the ability to specify actors for
  //  specific pipelines; i.e. only update the color actor or only the label
  //  actor, etc.
  for (const auto& actor : actors_.at(id)) {
    actor->SetUserTransform(vtk_X_WG);
  }
}

bool RenderEngineOspray::DoRemoveGeometry(GeometryId id) {
  auto iter = actors_.find(id);

  if (iter == actors_.end()) return false;

  std::array<vtkSmartPointer<vtkActor>, kNumPipelines>& pipe_actors =
      iter->second;
  for (int i = 0; i < kNumPipelines; ++i) {
    // If the label actor hasn't been added to its renderer, this is a no-op.
    pipelines_[i]->renderer->RemoveActor(pipe_actors[i]);
  }
  actors_.erase(iter);
  return true;
}

std::unique_ptr<RenderEngine> RenderEngineOspray::DoClone() const {
  // Note: we can't use make_unique because the copy constructor is private.
  return std::unique_ptr<RenderEngineOspray>(new RenderEngineOspray(*this));
}

// Note: this is a private copy constructor implemented solely to facilitate
// cloning. This code should simply be rolled into DoClone(). (See the TODO
// in the header file.)
RenderEngineOspray::RenderEngineOspray(const RenderEngineOspray& other)
    : RenderEngine(other),
      pipelines_{{make_unique<RenderingPipeline>()}},
      default_diffuse_{other.default_diffuse_},
      background_color_{other.background_color_},
      render_mode_(other.render_mode_) {
  InitializePipelines(other.get_params().samples_per_pixel);

  // Utility function for creating a cloned actor which *shares* the same
  // underlying polygonal data.
  auto clone_actor_array =
      [this](const std::array<vtkSmartPointer<vtkActor>, kNumPipelines>&
                 source_actors,
             std::array<vtkSmartPointer<vtkActor>, kNumPipelines>*
                 clone_actors_ptr) {
        DRAKE_DEMAND(clone_actors_ptr != nullptr);
        std::array<vtkSmartPointer<vtkActor>, kNumPipelines>& clone_actors =
            *clone_actors_ptr;
        for (int i = 0; i < kNumPipelines; ++i) {
          // NOTE: source *should* be const; but none of the getters on the
          // source are const-compatible.
          DRAKE_DEMAND(source_actors[i]);
          DRAKE_DEMAND(clone_actors[i]);
          vtkActor& source = *source_actors[i];
          vtkActor& clone = *clone_actors[i];

          // TODO(SeanCurtis-TRI): Modify this once OSPRay-specific materials
          //  are supported.
          if (source.GetTexture() == nullptr) {
            clone.GetProperty()->SetColor(source.GetProperty()->GetColor());
            clone.GetProperty()->SetOpacity(source.GetProperty()->GetOpacity());
          } else {
            clone.SetTexture(source.GetTexture());
          }

          // NOTE: The clone renderer and original renderer *share* polygon
          // data. If the meshes were *deformable* this would be invalid.
          // Furthermore, even if dynamic adding/removing of geometry were
          // valid, VTK's reference counting preserves the underlying geometry
          // in the copy that still references it.
          clone.SetMapper(source.GetMapper());
          clone.SetUserTransform(source.GetUserTransform());

          pipelines_.at(i)->renderer.Get()->AddActor(&clone);
        }
      };

  for (const auto& other_id_actor_pair : other.actors_) {
    std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
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

  // TODO(SeanCurtis-TRI): Copy light.
}

void RenderEngineOspray::InitializePipelines(int samples_per_pixel) {
  const vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(RigidTransformd::Identity());

  // TODO(SeanCurtis-TRI): Things like configuring lights should *not* be part
  //  of initializing the pipelines. When we support light declaration, this
  //  will get moved out.
  light_->SetLightTypeToCameraLight();
  light_->SetConeAngle(45.0);
  light_->SetAttenuationValues(1.0, 0.0, 0.0);
  light_->SetIntensity(1);
  // OSPRay specific control, radius to get soft shadows.
  if (render_mode_ == OsprayMode::kPathTracer) {
    vtkOSPRayLightNode::SetRadius(1.0, light_);
  }
  light_->SetTransformMatrix(vtk_identity->GetMatrix());

  // Generic configuration of pipelines.
  for (auto& pipeline : pipelines_) {
    // OSPRay specific configuration.
    pipeline->renderer->SetPass(ospray_);
    if (render_mode_ == OsprayMode::kRayTracer) {
      vtkOSPRayRendererNode::SetRendererType("scivis", pipeline->renderer);
      vtkOSPRayRendererNode::SetAmbientSamples(0, pipeline->renderer);
      pipeline->renderer->UseShadowsOn();
      // NOTE: It appears that ospray does [0, 1] -> [0, 255] conversion via
      // truncation. So, to affect rounding, we have to bump the background
      // color by half a bit so that it rounds properly.
      const double delta = 0.5 / 255;
      ColorD bg{background_color_.r + delta, background_color_.g + delta,
                background_color_.b + delta};
      pipeline->renderer->SetBackground(bg.r, bg.g, bg.b);
    } else {
      vtkOSPRayRendererNode::SetRendererType("pathtracer", pipeline->renderer);
      vtkOSPRayRendererNode::SetSamplesPerPixel(samples_per_pixel,
                                                pipeline->renderer);
      // TODO(SeanCurtis-TRI): When our VTK library has been updated to include
      //  the denoiser introduced in
      //  https://gitlab.kitware.com/vtk/vtk/merge_requests/5297
      //  make the denoiser and its threshold parameter available.
      //  vtkOSPRayRendererNode::ENABLE_DENOISER();
      //  vtkOSPRayRendererNode::SetEnableDenoiser(4, pipeline->renderer);
    }
    double np[3] = {0, 0, 1};
    double ep[3] = {0, 1, 0};
    vtkOSPRayRendererNode::SetNorthPole(np, pipeline->renderer);
    vtkOSPRayRendererNode::SetEastPole(ep, pipeline->renderer);

    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(90.0);  // Default to an arbitrary 90° field of view.
    SetModelTransformMatrixToVtkCamera(camera, vtk_identity);

    pipeline->window->AddRenderer(pipeline->renderer.GetPointer());
    pipeline->filter->SetInput(pipeline->window.GetPointer());
    pipeline->filter->SetScale(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();

    pipeline->renderer->AddLight(light_);
  }
}

void RenderEngineOspray::ImplementObj(const std::string& file_name,
                                      double scale, void* user_data) {
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

void RenderEngineOspray::ImplementGeometry(vtkPolyDataAlgorithm* source,
                                           void* user_data) {
  DRAKE_DEMAND(user_data != nullptr);

  std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
      vtkSmartPointer<vtkActor>::New()};
  // Note: the mappers ultimately get referenced by the actors, so they do _not_
  // get destroyed when this array goes out of scope.
  std::array<vtkNew<vtkOpenGLPolyDataMapper>, kNumPipelines> mappers;

  for (auto& mapper : mappers) {
    mapper->SetInputConnection(source->GetOutputPort());
  }

  const RegistrationData& data =
      *reinterpret_cast<RegistrationData*>(user_data);

  // If the geometry is anchored, X_FG = X_WG so I'm setting the pose for
  // anchored geometry -- for all other values of F, it is dynamic and will be
  // re-written in the first pose update.
  vtkSmartPointer<vtkTransform> vtk_X_PG = ConvertToVtkTransform(data.X_FG);

  // Adds the actor into the specified pipeline.
  auto connect_actor = [this, &actors, &mappers,
      &vtk_X_PG](ImageType image_type) {
    actors[image_type]->SetMapper(mappers[image_type].Get());
    actors[image_type]->SetUserTransform(vtk_X_PG);
    pipelines_[image_type]->renderer->AddActor(actors[image_type].Get());
  };

  // Color actor.
  auto& color_actor = actors[ImageType::kColor];

  // TODO(SeanCurtis-TRI): Modify this once OSPRay-specific materials are
  //  supported.
  const std::string& diffuse_map_name =
      data.properties.GetPropertyOrDefault<std::string>("phong", "diffuse_map",
                                                        "");
  // Legacy support for *implied* texture maps. If we have mesh.obj, we look for
  // mesh.png (unless one has been specifically called out in the properties).
  // TODO(SeanCurtis-TRI): Remove this legacy texture when objects and materials
  // are coherently specified by SDF/URDF/obj/mtl, etc.
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
    const Vector4d& diffuse =
        data.properties.GetPropertyOrDefault("phong", "diffuse",
                                             default_diffuse_);
    color_actor->GetProperty()->SetColor(diffuse(0), diffuse(1), diffuse(2));
    color_actor->GetProperty()->SetOpacity(diffuse(3));
  }

  connect_actor(ImageType::kColor);

  // Take ownership of the actors.
  actors_.insert({data.id, std::move(actors)});
}

void RenderEngineOspray::PerformVtkUpdate(const RenderingPipeline& p) {
  p.window->Render();
  // See the note in the VTK documentation about explicitly calling Modified
  // on the filter:
  // https://vtk.org/doc/nightly/html/classvtkWindowToImageFilter.html#details
  p.filter->Modified();
  p.filter->Update();
}

void RenderEngineOspray::UpdateWindow(const CameraProperties& camera,
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

}  // namespace render
}  // namespace geometry
}  // namespace drake

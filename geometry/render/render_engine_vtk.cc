#include "drake/geometry/render/render_engine_vtk.h"

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

#include "drake/geometry/render/shaders/depth_shaders.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/vtk_util.h"

namespace drake {
namespace geometry {
namespace render {

using Eigen::Vector4d;
using std::make_unique;
using math::RigidTransformd;
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

// A note on OpenGL clipping planes and DepthCameraProperties near and far.
// The z near and far planes reported in DepthCameraProperties are properties of
// the modeled sensor. They cannot produce reliable depths closer than z_near
// or farther than z_far.
// We *could* set the OpenGl clipping planes to [z_near, z_far], however, that
// will lead to undesirable artifacts. Any geometry that lies at a distance
// in the range [0, z_near) would be clipped away with *no* occluding effects.
// So, instead, we render the depth image in the range [epsilon, z_far] and
// then detect the occluding pixels that lie closer than the camera's supported
// range and mark them as too close. Clipping all geometry beyond z_far is not
// a problem because they can unambiguously be marked as too far.
const double kClippingPlaneNear = 0.01;
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
  // [kClippingPlaneNear, z_far]. If the buffer value is *not* 1,
  // then it lies inside the range.
  float z = std::numeric_limits<float>::quiet_NaN();
  if (z_buffer_value != 1.f) {
    z = static_cast<float>(z_buffer_value * kA + kClippingPlaneNear);
    // TODO(SeanCurtis-TRI): This is now slightly strange; the OpenGL clipping
    //  plane is being set to the camera's z_far; we should never get a value
    //  greater than that. Clean this up. Ideally, do more in the shader.
    if (z > z_far) {
      z = InvalidDepth::kTooFar;
    } else if (z < z_near) {
      z = InvalidDepth::kTooClose;
    }
  }
  return z;
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

namespace internal {

ShaderCallback::ShaderCallback() :
    z_near_(kClippingPlaneNear),
    // This value will be overwritten by the camera's z_far value.
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

void RenderEngineVtk::RenderColorImage(const CameraProperties& camera,
                                       bool show_window,
                                       ImageRgba8U* color_image_out) const {
  UpdateWindow(camera, show_window, pipelines_[ImageType::kColor].get(),
               "Color Image");
  PerformVtkUpdate(*pipelines_[ImageType::kColor]);

  // TODO(SeanCurtis-TRI): Determine if this copies memory (and find some way
  // around copying).
  pipelines_[ImageType::kColor]->exporter->Export(color_image_out->at(0, 0));
}

void RenderEngineVtk::RenderDepthImage(const DepthCameraProperties& camera,
                                       ImageDepth32F* depth_image_out) const {
  UpdateWindow(camera, pipelines_[ImageType::kDepth].get());
  PerformVtkUpdate(*pipelines_[ImageType::kDepth]);

  // TODO(SeanCurtis-TRI): This copies the image and *that's* a tragedy. It
  // would be much better to process the pixels directly. The solution is to
  // simply call exporter->GetPointerToData() and process the pixels myself.
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
                                       bool show_window,
                                       ImageLabel16I* label_image_out) const {
  UpdateWindow(camera, show_window, pipelines_[ImageType::kLabel].get(),
               "Label Image");
  PerformVtkUpdate(*pipelines_[ImageType::kLabel]);

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
      label_image_out->at(u, v)[0] = RenderEngine::LabelFromColor(color);
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

bool RenderEngineVtk::DoRegisterVisual(
    GeometryId id, const Shape& shape, const PerceptionProperties& properties,
    const RigidTransformd& X_FG) {
  // Note: the user_data interface on reification requires a non-const pointer.
  RegistrationData data{properties, X_FG, id};
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

    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(90.0);  // Default to an arbitrary 90° field of view.
    // Initialize far plane to arbitrary value. In the case of depth it will be
    // overwritten by the depth camera's z-far value.
    // TODO(SeanCurtis-TRI): Provide mechanism where user can set this value.
    //  It's important to expose this as it will affect the efficacy of the
    //  z-buffer.
    camera->SetClippingRange(kClippingPlaneNear, 100.0);
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

void RenderEngineVtk::PerformVtkUpdate(const RenderingPipeline& p) {
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

  // See notes at the definition of kClippingPlaneNear (above) for the
  // explanation of this value.
  uniform_setting_callback_->set_z_near(kClippingPlaneNear);
  uniform_setting_callback_->set_z_far(static_cast<float>(camera.z_far));
  // Never show window for depth camera; it is a meaningless operation as the
  // raw depth rasterization is not human consummable.
  UpdateWindow(camera, false, p, "Depth Image");
}

}  // namespace render
}  // namespace geometry
}  // namespace drake

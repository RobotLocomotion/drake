#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkActor.h>                    // vtkRenderingCore
#include <vtkCamera.h>                   // vtkRenderingCore
#include <vtkCylinderSource.h>           // vtkFiltersSources
#include <vtkGLTFImporter.h>             // vtkIOImport
#include <vtkHDRReader.h>                // vtkIOImage
#include <vtkImageCast.h>                // vtkImagingCore
#include <vtkImageFlip.h>                // vtkImagingCore
#include <vtkImageReader2.h>             // vtkIOImage
#include <vtkImageReader2Factory.h>      // vtkIOImage
#include <vtkOpenGLPolyDataMapper.h>     // vtkRenderingOpenGL2
#include <vtkOpenGLRenderer.h>           // vtkRenderingOpenGL2
#include <vtkOpenGLShaderProperty.h>     // vtkRenderingOpenGL2
#include <vtkOpenGLTexture.h>            // vtkRenderingOpenGL2
#include <vtkPNGReader.h>                // vtkIOImage
#include <vtkPlaneSource.h>              // vtkFiltersSources
#include <vtkProperty.h>                 // vtkRenderingCore
#include <vtkSkybox.h>                   // vtkRenderingCore
#include <vtkTexture.h>                  // vtkRenderingCore
#include <vtkTexturedSphereSource.h>     // vtkFiltersSources
#include <vtkTransform.h>                // vtkCommonTransforms
#include <vtkTransformPolyDataFilter.h>  // vtkFiltersGeneral

#include "drake/common/diagnostic_policy.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/render/shaders/depth_shaders.h"
#include "drake/geometry/render_vtk/internal_render_engine_vtk_base.h"
#include "drake/geometry/render_vtk/internal_vtk_util.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/vtk_diagnostic_event_observer.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::internal::DefineMaterial;
using geometry::internal::LoadRenderMeshesFromObj;
using geometry::internal::RenderMaterial;
using geometry::internal::RenderMesh;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
using std::make_unique;
using systems::sensors::CameraInfo;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;
using systems::sensors::internal::VtkDiagnosticEventObserver;

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

// Reports the texture to use for the environment map with some important
// details regarding how the environment map should be configured.
struct EnvironmentTexture {
  vtkSmartPointer<vtkTexture> texture;
  bool is_hdr{};
};

// Taken from: https://examples.vtk.org/site/Cxx/Rendering/PBR_HDR_Environment/
EnvironmentTexture ReadEquirectangularFile(std::string const& fileName) {
  vtkNew<vtkTexture> texture;

  std::string extension =
      std::filesystem::path(fileName).extension().generic_string();
  std::transform(extension.cbegin(), extension.cend(), extension.begin(),
                 [](char c) {
                   return std::tolower(c);
                 });

  bool is_hdr = false;
  if (std::string(".jpeg .jpg .png").find(extension, 0) != std::string::npos) {
    vtkNew<vtkImageReader2Factory> readerFactory;
    vtkSmartPointer<vtkImageReader2> imgReader;
    imgReader.TakeReference(
        readerFactory->CreateImageReader2(fileName.c_str()));
    imgReader->SetFileName(fileName.c_str());
    texture->SetInputConnection(imgReader->GetOutputPort());
  } else {
    vtkNew<vtkHDRReader> reader;
    auto extensions = reader->GetFileExtensions();
    if (std::string(extensions).find(extension, 0) != std::string::npos) {
      if (reader->CanReadFile(fileName.c_str())) {
        reader->SetFileName(fileName.c_str());

        texture->SetInputConnection(reader->GetOutputPort());
        texture->SetColorModeToDirectScalars();
        is_hdr = true;
      } else {
        throw std::runtime_error(fmt::format(
            "Unable to instantiate environment map for RenderEngineVtk: '{}'.",
            fileName));
      }
    }
  }

  texture->MipmapOn();
  texture->InterpolateOn();

  return {texture, is_hdr};
}
}  // namespace

ShaderCallback::ShaderCallback()
    :  // These values are arbitrary "reasonable" values, but we expect them to
       // *both* be overwritten upon every usage.
      z_near_(0.01),
      z_far_(100.0) {}

vtkNew<ShaderCallback> RenderEngineVtk::uniform_setting_callback_;

RenderEngineVtk::RenderEngineVtk(const RenderEngineVtkParams& parameters)
    : RenderEngine(RenderLabel::kDontCare),
      parameters_(parameters),
      pipelines_{{make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>()}} {
  // Only populate the fallback lights if we haven't specified an environment
  // map.
  // Until we introduce CubeMap, the default texture (NullTexture) should be
  // treated as if no environment map has been provided.
  if (!parameters.environment_map.has_value() ||
      parameters.environment_map->texture.index() == 0) {
    fallback_lights_.push_back({});
  }
  if (parameters.default_diffuse) {
    default_diffuse_.set(*parameters.default_diffuse);
  }
  default_clear_color_.set(parameters.default_clear_color);

  InitializePipelines();
}

void RenderEngineVtk::UpdateViewpoint(const RigidTransformd& X_WC) {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (const auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RenderEngineVtk::ImplementGeometry(const Box& box, void* user_data) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  ImplementPolyData(CreateVtkBox(box, data.properties).GetPointer(),
                    DefineMaterial(data.properties, default_diffuse_), data);
}

void RenderEngineVtk::ImplementGeometry(const Capsule& capsule,
                                        void* user_data) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  ImplementPolyData(CreateVtkCapsule(capsule).GetPointer(),
                    DefineMaterial(data.properties, default_diffuse_), data);
}

void RenderEngineVtk::ImplementGeometry(const Convex& convex, void* user_data) {
  ImplementMesh(convex.filename(), convex.scale(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Cylinder& cylinder,
                                        void* user_data) {
  vtkNew<vtkCylinderSource> vtk_cylinder;
  SetCylinderOptions(vtk_cylinder, cylinder.length(), cylinder.radius());
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  ImplementPolyData(TransformToDrakeCylinder(vtk_cylinder),
                    DefineMaterial(data.properties, default_diffuse_), data);
}

void RenderEngineVtk::ImplementGeometry(const Ellipsoid& ellipsoid,
                                        void* user_data) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  ImplementPolyData(CreateVtkEllipsoid(ellipsoid).GetPointer(),
                    DefineMaterial(data.properties, default_diffuse_), data);
}

void RenderEngineVtk::ImplementGeometry(const HalfSpace&, void* user_data) {
  vtkSmartPointer<vtkPlaneSource> vtk_plane = CreateSquarePlane(kTerrainSize);

  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  ImplementPolyData(vtk_plane.GetPointer(),
                    DefineMaterial(data.properties, default_diffuse_), data);
}

void RenderEngineVtk::ImplementGeometry(const Mesh& mesh, void* user_data) {
  ImplementMesh(mesh.filename(), mesh.scale(), user_data);
}

void RenderEngineVtk::ImplementGeometry(const Sphere& sphere, void* user_data) {
  vtkNew<vtkTexturedSphereSource> vtk_sphere;
  SetSphereOptions(vtk_sphere.GetPointer(), sphere.radius());
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  ImplementPolyData(vtk_sphere.GetPointer(),
                    DefineMaterial(data.properties, default_diffuse_), data);
}

bool RenderEngineVtk::DoRegisterVisual(GeometryId id, const Shape& shape,
                                       const PerceptionProperties& properties,
                                       const RigidTransformd& X_WG) {
  // Note: the user_data interface on reification requires a non-const pointer.
  RegistrationData data{properties, X_WG, id};
  shape.Reify(this, &data);
  return data.accepted;
}

void RenderEngineVtk::DoUpdateVisualPose(GeometryId id,
                                         const RigidTransformd& X_WG) {
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(X_WG);
  // TODO(SeanCurtis-TRI): Perhaps provide the ability to specify which pipeline
  //  is being updated and only update the pose of the prop for that pipeline.
  for (const auto& prop : props_.at(id)) {
    for (const auto& part : prop.parts) {
      if (part.T_GA != nullptr) {
        // The goal is to update the actor's pose without allocating new
        // transforms or matrices. Using part.actor->GetUserMatrix() as the
        // result of the matrix product is unreliable. Instead, write to the
        // matrix from the user transform.
        vtkMatrix4x4* T_WA = part.actor->GetUserTransform()->GetMatrix();
        vtkMatrix4x4::Multiply4x4(vtk_X_WG->GetMatrix(), part.T_GA, T_WA);
        part.actor->Modified();
      } else {
        part.actor->SetUserTransform(vtk_X_WG);
      }
    }
  }
}

bool RenderEngineVtk::DoRemoveGeometry(GeometryId id) {
  auto iter = props_.find(id);

  if (iter != props_.end()) {
    PropArray& pipe_props = iter->second;
    for (int i = 0; i < kNumPipelines; ++i) {
      for (const auto& part : pipe_props[i].parts) {
        pipelines_[i]->renderer->RemoveActor(part.actor);
      }
    }
    props_.erase(iter);
    return true;
  }

  return false;
}

std::unique_ptr<RenderEngine> RenderEngineVtk::DoClone() const {
  return std::unique_ptr<RenderEngineVtk>(new RenderEngineVtk(*this));
}

void RenderEngineVtk::DoRenderColorImage(const ColorRenderCamera& camera,
                                         ImageRgba8U* color_image_out) const {
  UpdateWindow(camera.core(), camera.show_window(),
               *pipelines_[ImageType::kColor], "Color Image");
  PerformVtkUpdate(*pipelines_[ImageType::kColor]);

  // TODO(SeanCurtis-TRI): Determine if this copies memory (and find some way
  // around copying).
  pipelines_[ImageType::kColor]->exporter->Export(color_image_out->at(0, 0));
}

void RenderEngineVtk::DoRenderDepthImage(const DepthRenderCamera& camera,
                                         ImageDepth32F* depth_image_out) const {
  UpdateWindow(camera, *pipelines_[ImageType::kDepth]);
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
        depth_image_out->at(u, v)[0] =
            CheckRangeAndConvertToMeters(shader_value, min_depth, max_depth);
      }
    }
  }
}

void RenderEngineVtk::DoRenderLabelImage(const ColorRenderCamera& camera,
                                         ImageLabel16I* label_image_out) const {
  UpdateWindow(camera.core(), camera.show_window(),
               *pipelines_[ImageType::kLabel], "Label Image");
  PerformVtkUpdate(*pipelines_[ImageType::kLabel]);

  // TODO(SeanCurtis-TRI): This copies the image and *that's* a tragedy. It
  // would be much better to process the pixels directly. The solution is to
  // simply call exporter->GetPointerToData() and process the pixels myself.
  // See the implementation in vtkImageExport::Export() for details.
  const CameraInfo& intrinsics = camera.core().intrinsics();
  ImageRgba8U image(intrinsics.width(), intrinsics.height());
  pipelines_[ImageType::kLabel]->exporter->Export(image.at(0, 0));

  for (int v = 0; v < intrinsics.height(); ++v) {
    for (int u = 0; u < intrinsics.width(); ++u) {
      label_image_out->at(u, v)[0] = RenderEngine::MakeLabelFromRgb(
          image.at(u, v)[0], image.at(u, v)[1], image.at(u, v)[2]);
    }
  }
}

RenderEngineVtk::RenderEngineVtk(const RenderEngineVtk& other)
    : RenderEngine(other),
      parameters_(other.parameters_),
      pipelines_{{make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>(),
                  make_unique<RenderingPipeline>()}},
      default_diffuse_{other.default_diffuse_},
      default_clear_color_{other.default_clear_color_},
      fallback_lights_(other.fallback_lights_) {
  InitializePipelines();

  for (const auto& [id, source_props] : other.props_) {
    PropArray target_props;
    for (int i = 0; i < kNumPipelines; ++i) {
      auto& renderer = *pipelines_.at(i)->renderer;
      const Prop& source_prop = source_props[i];
      Prop& target_prop = target_props[i];
      for (const auto& source_part : source_prop.parts) {
        vtkNew<vtkActor> target_actor;
        target_actor->ShallowCopy(source_part.actor);
        renderer.AddActor(target_actor);
        target_prop.parts.push_back(
            Part{.actor = std::move(target_actor), .T_GA = source_part.T_GA});
      }
    }
    props_.insert({id, std::move(target_props)});
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

void RenderEngineVtk::ImplementMesh(const std::string& file_name, double scale,
                                    void* user_data) {
  auto& data = *static_cast<RegistrationData*>(user_data);

  const std::string extension = Mesh(file_name).extension();
  if (extension == ".obj") {
    data.accepted = ImplementObj(file_name, scale, data);
  } else if (extension == ".gltf") {
    data.accepted = ImplementGltf(file_name, scale, data);
  } else {
    static const logging::Warn one_time(
        "RenderEngineVtk only supports Mesh/Convex specifications which use "
        ".obj and .gltf files. Mesh specifications using other mesh types "
        "(e.g., .stl, .dae, etc.) will be ignored.");
    data.accepted = false;
  }
}

bool RenderEngineVtk::ImplementObj(const std::string& file_name, double scale,
                                   const RegistrationData& data) {
  std::vector<RenderMesh> meshes =
      LoadRenderMeshesFromObj(file_name, data.properties, default_diffuse_,
                              drake::internal::DiagnosticPolicy());
  for (auto& render_mesh : meshes) {
    const RenderMaterial material = render_mesh.material;

    vtkSmartPointer<vtkPolyDataAlgorithm> mesh_source =
        CreateVtkMesh(std::move(render_mesh));

    if (scale == 1) {
      ImplementPolyData(mesh_source.GetPointer(), material, data);
      continue;
    }

    vtkNew<vtkTransform> transform;
    // TODO(SeanCurtis-TRI): Should I be allowing only isotropic scale.
    transform->Scale(scale, scale, scale);
    vtkNew<vtkTransformPolyDataFilter> transform_filter;
    transform_filter->SetInputConnection(mesh_source->GetOutputPort());
    transform_filter->SetTransform(transform.GetPointer());
    transform_filter->Update();

    ImplementPolyData(transform_filter.GetPointer(), material, data);
  }
  return true;
}

bool RenderEngineVtk::ImplementGltf(const std::string& file_name, double scale,
                                    const RegistrationData& data) {
  // TODO(SeanCurtis-TRI): introduce VtkDiagnosticEventObserver on the gltf
  // importer (see systems/sensors/image_io_load.cc).
  vtkNew<vtkGLTFImporter> importer;
  importer->SetFileName(file_name.c_str());
  importer->Update();

  auto* renderer = importer->GetRenderer();
  DRAKE_DEMAND(renderer != nullptr);

  if (renderer->VisibleActorCount() == 0) {
    log()->warn("No visible meshes found in glTF file: {}", file_name);
    return false;
  }

  // The pose of the geometry in Drake's world. This is a component of the final
  // UserMatrix value.
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(data.X_WG);

  // The relative transform from the file's frame F to the geometry's frame G.
  // This includes the rotation from y-up to z-up and the requested scale.
  const RigidTransformd X_GF(RotationMatrixd::MakeXRotation(M_PI / 2));
  vtkSmartPointer<vtkTransform> T_GF_transform =
      ConvertToVtkTransform(X_GF, scale);
  vtkMatrix4x4* T_GF = T_GF_transform->GetMatrix();

  // Color.
  if (data.properties.HasProperty("phong", "diffuse") ||
      data.properties.HasProperty("phong", "diffuse_map")) {
    log()->warn(
        "Drake materials have been assigned to a glTF file. glTF defines its "
        "own materials, so post hoc materials will be ignored and should be "
        "removed from the model specification. glTF file: '{}'",
        file_name);
  }

  const RenderLabel label = GetRenderLabelOrThrow(data.properties);
  const Rgba label_color = RenderEngine::MakeRgbFromLabel(label);

  // The final assemblies associated with the GeometryId.
  PropArray prop_array;

  for (int i = 0; i < kNumPipelines; ++i) {
    // VTK imports a collection of visible parts (nee vtkActors), regardless
    // what the glTF hierarchy was, The actor poses are all measured and
    // expressed w.r.t. the file's frame F.
    //
    // We'll store each actor from the glTF file in a single Part, storing the
    // transform of the actor frame A to the *geometry* frame G (see below).
    auto& prop = prop_array[i];
    auto* actors = renderer->GetActors();
    actors->InitTraversal();
    // For each source_actor, create a color, depth, and label actor.
    while (vtkActor* source_actor = actors->GetNextActor()) {
      vtkNew<vtkActor> part_actor;
      if (i == ImageType::kColor) {
        // Color rendering can use the source_actor without changes.
        part_actor->ShallowCopy(source_actor);
      } else {
        // Depth and label images require new actors, based on the source, but
        // with changes to their materials (aka "mapper").
        vtkNew<vtkOpenGLPolyDataMapper> mapper;
        part_actor->SetMapper(mapper);
        mapper->SetInputConnection(
            source_actor->GetMapper()->GetInputAlgorithm()->GetOutputPort());
        if (i == ImageType::kLabel) {
          // Label requires a mapper with the encoded RenderLabel color.
          part_actor->GetProperty()->LightingOff();
          part_actor->GetProperty()->SetColor(label_color.r(), label_color.g(),
                                              label_color.b());
        } else if (i == ImageType::kDepth) {
          // Depth requires a mapper with the depth shader.
          vtkOpenGLShaderProperty* shader_prop =
              vtkOpenGLShaderProperty::SafeDownCast(
                  part_actor->GetShaderProperty());
          DRAKE_DEMAND(shader_prop != nullptr);
          shader_prop->SetVertexShaderCode(render::shaders::kDepthVS);
          shader_prop->SetFragmentShaderCode(render::shaders::kDepthFS);
          mapper->AddObserver(vtkCommand::UpdateShaderEvent,
                              uniform_setting_callback_.Get());
        }
      }
      // vtkGLTFImporter uses the actor's UserTransform property to define the
      // actor's transform in the file frame (T_FA). We also have to use the
      // UserTransform to pose the geometry in Drake's world (T_WG). So, we will
      // interpret UserTransform as T_WA = X_WG * T_GA = X_WG * T_GF * T_FA.
      // We save T_GA with the actor in the part so we can keep computing T_WA
      // as X_WG changes.
      vtkNew<vtkMatrix4x4> T_GA;
      vtkMatrix4x4* T_FA = source_actor->GetUserMatrix();
      vtkMatrix4x4::Multiply4x4(T_GF, T_FA, T_GA);
      vtkNew<vtkMatrix4x4> T_WA;
      vtkMatrix4x4::Multiply4x4(vtk_X_WG->GetMatrix(), T_GA, T_WA);
      part_actor->SetUserMatrix(T_WA);
      pipelines_.at(i)->renderer->AddActor(part_actor);
      prop.parts.push_back(
          {.actor = std::move(part_actor), .T_GA = std::move(T_GA)});
    }
  }

  props_.insert({data.id, std::move(prop_array)});

  // Successfully parsing a glTF should require all materials to be PBR.
  SetPbrMaterials();

  // We've successfully processed the .gltf. Report it as accepted.
  return true;
}

namespace {

vtkSmartPointer<vtkLight> MakeVtkLight(const LightParameter& light_param) {
  vtkNew<vtkLight> light;
  light->SetColor(light_param.color.rgba().data());
  light->SetIntensity(light_param.intensity);
  if (light_param.type == "directional") {
    light->SetPositional(false);
  } else {
    light->SetPositional(true);
    light->SetAttenuationValues(light_param.attenuation_values.data());
    if (light_param.type == "point") {
      light->SetConeAngle(90.0);
    } else if (light_param.type == "spot") {
      light->SetConeAngle(light_param.cone_angle);
    } else {
      throw std::runtime_error(
          fmt::format("RenderEngineVtk given an invalid light type. Expected "
                      "one of 'point', 'spot', or 'directional'. Given '{}'.",
                      light_param.type));
    }
  }
  if (light_param.frame == "camera") {
    // Drake's camera frame C has the camera looking in the Cz direction (with
    // Cx pointing right in the image and Cy down). The light parameters are
    // expressed in that frame. VTK's camera frame V relates to C as follows:
    // Cx = Vx, Cy = -Vy, and Cz = -Vz, with p_CoVo_V = Vz (i.e., [0, 0, 1]).
    // We need to configure the lights in VTK's frame V.
    const Vector3d& p_CL_C = light_param.position;
    const Vector3d p_CL_D(p_CL_C.x(), -p_CL_C.y(), -p_CL_C.z() + 1);
    light->SetPosition(p_CL_D.data());
    const Vector3d& dir_LT_C = light_param.direction.normalized();
    const Vector3d& dir_LT_D{dir_LT_C.x(), -dir_LT_C.y(), -dir_LT_C.z()};
    const Vector3d p_CT_D = p_CL_D + dir_LT_D;
    // Setting the focal point on a point light is harmless.
    light->SetFocalPoint(p_CT_D.data());
    light->SetLightTypeToCameraLight();
  } else if (light_param.frame == "world") {
    light->SetPosition(light_param.position.data());
    const Vector3d p_WT =
        light_param.position + light_param.direction.normalized();
    light->SetFocalPoint(p_WT.data());
    light->SetLightTypeToSceneLight();
  } else {
    throw std::runtime_error(
        fmt::format("RenderEngineVtk given an invalid frame for a light. "
                    "Expected one of 'world' or 'camera'. Given '{}'.",
                    light_param.frame));
  }

  return light;
}

}  // namespace

void RenderEngineVtk::InitializePipelines() {
  const vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(RigidTransformd::Identity());

  // Generic configuration of pipelines.
  for (auto& pipeline : pipelines_) {
    // When VTK experiences a warning, send it to drake::log()->warn().
    // When VTK experiences an error, throw it as an exception.
    vtkNew<VtkDiagnosticEventObserver> observer;
    observer->set_diagnostic(&diagnostic_);
    auto observe = [&observer](const auto& vtk_object) {
      vtk_object->AddObserver(vtkCommand::ErrorEvent, observer);
      vtk_object->AddObserver(vtkCommand::WarningEvent, observer);
    };
    observe(pipeline->renderer);
    observe(pipeline->window);
    observe(pipeline->filter);
    observe(pipeline->exporter);

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
  }

  // Pipeline-specific tweaks.

  // Depth image background color is white -- the representation of the maximum
  // distance (e.g., infinity).
  pipelines_[ImageType::kDepth]->renderer->SetBackground(1., 1., 1.);

  const Rgba empty_color = RenderEngine::MakeRgbFromLabel(RenderLabel::kEmpty);
  pipelines_[ImageType::kLabel]->renderer->SetBackground(
      empty_color.r(), empty_color.g(), empty_color.b());

  vtkOpenGLRenderer* renderer =
      vtkOpenGLRenderer::SafeDownCast(pipelines_[ImageType::kColor]->renderer);
  renderer->SetUseDepthPeeling(1);
  renderer->UseFXAAOn();
  renderer->SetBackground(default_clear_color_.r(), default_clear_color_.g(),
                          default_clear_color_.b());
  renderer->SetBackgroundAlpha(1.0);
  // The only lights we add are this renderer's "active" lights.
  renderer->RemoveAllLights();
  renderer->AutomaticLightCreationOff();
  for (const auto& light_param : active_lights()) {
    renderer->AddLight(MakeVtkLight(light_param));
  }
  if (parameters_.environment_map.has_value()) {
    // Until we have a CubeMap, the zero-index represents the default value of
    // "no texture specified". So, we'll simply return.
    if (parameters_.environment_map->texture.index() == 0) {
      log()->warn(
          "RenderEngineVtk has been configured to use an environment map, but "
          "no equirectangular texture has been provided.");
      return;
    }
    const std::string& path =
        std::get<EquirectangularMap>(parameters_.environment_map->texture).path;
    EnvironmentTexture env_map = ReadEquirectangularFile(path);
    renderer->UseImageBasedLightingOn();
    renderer->SetUseSphericalHarmonics(env_map.is_hdr);
    renderer->SetEnvironmentTexture(env_map.texture,
                                    /* isSRGB = */ !env_map.is_hdr);
    renderer->SetEnvironmentUp(0, 0, 1);
    if (parameters_.environment_map->skybox) {
      vtkNew<vtkSkybox> skybox;
      skybox->SetTexture(env_map.texture);
      skybox->SetFloorPlane(0, 0, 1, 0);
      // Note: it is *not* clear why setting the floor's right direction to -y
      // is necessary. However, without it, the skybox is not aligned with the
      // environment map.
      skybox->SetFloorRight(0, -1, 0);
      skybox->SetProjection(vtkSkybox::Sphere);
      // Linear color space (aka *not HDR*) requires gamma correction.
      skybox->SetGammaCorrect(!env_map.is_hdr);
      renderer->AddActor(skybox);
    }
    // Setting an environment map should require all materials to be PBR.
    SetPbrMaterials();
  }
}

void RenderEngineVtk::ImplementPolyData(vtkPolyDataAlgorithm* source,
                                        const RenderMaterial& material,
                                        const RegistrationData& data) {
  std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
      vtkSmartPointer<vtkActor>::New(), vtkSmartPointer<vtkActor>::New(),
      vtkSmartPointer<vtkActor>::New()};
  // Note: the mappers ultimately get referenced by the actors, so they do _not_
  // get destroyed when this array goes out of scope.
  std::array<vtkNew<vtkOpenGLPolyDataMapper>, kNumPipelines> mappers;

  // Sets vertex and fragment shaders only to the depth mapper.
  vtkOpenGLShaderProperty* shader_prop = vtkOpenGLShaderProperty::SafeDownCast(
      actors[ImageType::kDepth]->GetShaderProperty());
  DRAKE_DEMAND(shader_prop != nullptr);
  shader_prop->SetVertexShaderCode(render::shaders::kDepthVS);
  shader_prop->SetFragmentShaderCode(render::shaders::kDepthFS);
  mappers[ImageType::kDepth]->AddObserver(vtkCommand::UpdateShaderEvent,
                                          uniform_setting_callback_.Get());

  for (auto& mapper : mappers) {
    mapper->SetInputConnection(source->GetOutputPort());
  }

  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(data.X_WG);

  // Adds the actor into the specified pipeline.
  PropArray props;
  auto connect_actor = [this, &actors, &mappers, &props,
                        &vtk_X_WG](ImageType image_type) {
    vtkSmartPointer<vtkActor>& actor = actors[image_type];
    actor->SetMapper(mappers[image_type].Get());
    actor->SetUserTransform(vtk_X_WG);
    pipelines_[image_type]->renderer->AddActor(actor);
    props[image_type].parts.push_back({.actor = actor, .T_GA = nullptr});
  };

  // Label actor.
  const RenderLabel label = GetRenderLabelOrThrow(data.properties);
  if (label != RenderLabel::kDoNotRender) {
    // NOTE: We only configure the label actor if it doesn't have to "do not
    // render" label applied. We *have* created an actor and connected it to
    // a mapper; but otherwise, we leave it disconnected.
    vtkActor* label_actor = actors[ImageType::kLabel].Get();
    // This is to disable shadows and to get an object painted with a single
    // color.
    label_actor->GetProperty()->LightingOff();
    const Rgba color = RenderEngine::MakeRgbFromLabel(label);
    label_actor->GetProperty()->SetColor(color.r(), color.g(), color.b());
    connect_actor(ImageType::kLabel);
  }

  // Color actor.
  vtkActor* color_actor = actors[ImageType::kColor].Get();
  if (use_pbr_materials_) {
    color_actor->GetProperty()->SetInterpolationToPBR();
  }
  if (!material.diffuse_map.empty()) {
    vtkNew<vtkPNGReader> texture_reader;
    texture_reader->SetFileName(material.diffuse_map.c_str());
    texture_reader->Update();
    if (texture_reader->GetOutput()->GetScalarType() != VTK_UNSIGNED_CHAR) {
      log()->warn(
          "Texture map '{}' has an unsupported bit depth, casting it to uchar "
          "channels.",
          material.diffuse_map.string());
    }

    vtkNew<vtkImageCast> caster;
    caster->SetOutputScalarType(VTK_UNSIGNED_CHAR);
    caster->SetInputConnection(texture_reader->GetOutputPort());
    caster->Update();
    DRAKE_DEMAND(caster->GetOutput() != nullptr);

    vtkNew<vtkOpenGLTexture> texture;
    texture->SetInputConnection(caster->GetOutputPort());
    // TODO(SeanCurtis-TRI): It doesn't seem like the scale is used to actually
    // *scale* the image.
    const Vector2d uv_scale = data.properties.GetPropertyOrDefault(
        "phong", "diffuse_scale", Vector2d{1, 1});
    const bool need_repeat = uv_scale[0] > 1 || uv_scale[1] > 1;
    texture->SetRepeat(need_repeat);
    texture->InterpolateOn();
    if (use_pbr_materials_) {
      texture->SetUseSRGBColorSpace(true);
      color_actor->GetProperty()->SetBaseColorTexture(texture);
    } else {
      color_actor->SetTexture(texture.Get());
    }
  }

  // Note: This allows the color map to be modulated by an arbitrary diffuse
  // color and opacity.
  const auto& diffuse = material.diffuse;
  color_actor->GetProperty()->SetColor(diffuse.r(), diffuse.g(), diffuse.b());
  color_actor->GetProperty()->SetOpacity(diffuse.a());

  connect_actor(ImageType::kColor);

  // Depth actor; always gets wired in with no additional work.
  connect_actor(ImageType::kDepth);

  // Take ownership of the actors.
  for (int i = 0; i < kNumPipelines; ++i) {
    // props was created in *this* method: one part per image type based on the
    // input poly data algorithm. Confirm it's only one. However, when added
    // into RenderEngineVtk::props_, it may be one of many parts for the same
    // geometry id.
    DRAKE_DEMAND(props[i].parts.size() == 1);
    props_[data.id][i].parts.push_back(std::move(props[i].parts[0]));
  }
}

RenderEngineVtk::RenderingPipeline& RenderEngineVtk::get_mutable_pipeline(
    internal::ImageType image_type) const {
  DRAKE_DEMAND(image_type == ImageType::kColor ||
               image_type == ImageType::kLabel ||
               image_type == ImageType::kDepth);
  return *pipelines_[image_type];
}

void RenderEngineVtk::SetDefaultLightPosition(const Vector3<double>&) {
  log()->warn(
      "RenderEngineVtk::SetDefaultLightPosition() no longer affects lighting. "
      "Instead, configure the lights at construction via "
      "RenderEngineVtkParams.");
}

void RenderEngineVtk::SetPbrMaterials() {
  if (!use_pbr_materials_) {
    use_pbr_materials_ = true;
    for (auto& [_, prop_array] : props_) {
      for (auto& part : prop_array[ImageType::kColor].parts) {
        part.actor->GetProperty()->SetInterpolationToPBR();
        if (part.actor->GetTexture() != nullptr &&
            part.actor->GetProperty()->GetTexture("albedoTex") == nullptr) {
          // Phong lighting uses the generic vtkActor "texture". PBR lighting
          // uses the "albedoTex" (sRGB) texture. We should promote the texture
          // as well.
          vtkTexture* texture = part.actor->GetTexture();
          texture->SetUseSRGBColorSpace(true);
          part.actor->GetProperty()->SetBaseColorTexture(
              part.actor->GetTexture());
        }
      }
    }
  }
}
void RenderEngineVtk::PerformVtkUpdate(const RenderingPipeline& p) {
  p.window->Render();
  p.filter->Modified();
  p.filter->Update();
}

void RenderEngineVtk::UpdateWindow(const RenderCameraCore& camera,
                                   bool show_window, const RenderingPipeline& p,
                                   const char* name) const {
  // NOTE: Although declared const, this method modifies VTK entities. The
  // conflict between ostensibly const operations and invocation of black-box
  // entities that need state mutated should be more formally handled.

  const CameraInfo& intrinsics = camera.intrinsics();
  p.window->SetSize(intrinsics.width(), intrinsics.height());
  p.window->SetOffScreenRendering(!show_window);
  if (show_window) p.window->SetWindowName(name);

  vtkCamera* vtk_camera = p.renderer->GetActiveCamera();
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
                                   const RenderingPipeline& p) const {
  uniform_setting_callback_->set_z_near(
      static_cast<float>(camera.depth_range().min_depth()));
  uniform_setting_callback_->set_z_far(
      static_cast<float>(camera.depth_range().max_depth()));
  // Never show window for depth camera; it is a meaningless operation as the
  // raw depth rasterization is not human consummable.
  UpdateWindow(camera.core(), false, p, "");
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake

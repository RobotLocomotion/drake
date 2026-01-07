#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <string_view>
#include <utility>
#include <vector>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <common_robotics_utilities/base64_helpers.hpp>
#include <vtkCamera.h>         // vtkRenderingCore
#include <vtkGLTFExporter.h>   // vtkIOExport
#include <vtkMatrix4x4.h>      // vtkCommonMath
#include <vtkVersionMacros.h>  // vtkCommonCore

#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/overloaded.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = std::filesystem;

using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::RenderCameraCore;
using render::RenderEngine;
using render_vtk::internal::ImageType;
using render_vtk::internal::RenderEngineVtk;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

/* Returns the next unique-per-process consecutive integer that can uniquely
 identify a scene.  Drake also provides an Identifier class to produce type-safe
 program-global IDs, but it's not guaranteed to be consecutive.
 See also:
 https://github.com/RobotLocomotion/drake/blob/master/common/identifier.h
 */
int64_t GetNextSceneId() {
  static never_destroyed<std::atomic<int64_t>> global_scene_id;
  return ++(global_scene_id.access());
}

/* RenderEngineGltfClient always produces a gltf+json file.  See also:
 https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#_media_type_registrations
 */
std::string MimeType() {
  return "model/gltf+json";
}

/* Drake uses an explicit projection matrix with RenderEngineVtk to fine tune
 the displayed viewing frustum tailored to the sensor being rendered.  The
 vtkGLTFExporter, however, will be querying the renderer's vtkCamera to export
 the relevant properties.  This method modifies the related camera fields that
 are exported for glTF.  As glTF has a limited definition for projection
 matrices, a Drake CameraInfo can't be fully expressed through its camera node.
 To remedy that, RenderClient::RenderOnServer() will emit the complete
 CameraInfo alongside a glTF scene file to prevent information loss.

 See "Notes on glTF Camera Specification" section on:
 https://drake.mit.edu/doxygen_cxx/group__render__engine__gltf__client__server__api.html

 See section 3.10.3 on glTF camera projection matrices:
 https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices

 NOTE: The modifications on the vtkCamera do *NOT* need to be undone, the camera
 is modified on every frame. */
void SetGltfCameraPerspective(const RenderCameraCore& core, vtkCamera* camera) {
  /* Exported parameters:
   https://gitlab.kitware.com/vtk/vtk/-/blob/v9.1.0/IO/Export/vtkGLTFExporter.cxx#L352

  camValues["znear"] = cam->GetClippingRange()[0];
  camValues["zfar"] = cam->GetClippingRange()[1];
  ...
  camValues["yfov"] = vtkMath::RadiansFromDegrees(cam->GetViewAngle());
  camValues["aspectRatio"] = ren->GetTiledAspectRatio();

  For aspectRatio, note that the vtkRenderer `ren`s aspect ratio will be
  computed via its vtkWindow member (RenderEngineVtk::UpdateWindow already calls
  SetSize on the associated vtkWindow).

  https://gitlab.kitware.com/vtk/vtk/-/blob/v9.1.0/Rendering/Core/vtkViewport.cxx#L667
   */
  camera->ParallelProjectionOff();  // Guarantee perspective (not orthographic).
  camera->SetClippingRange(core.clipping().near(), core.clipping().far());
  // VTK wants the view angle in degrees.
  const float fy = core.intrinsics().fov_y() * 180.0 / M_PI;
  camera->SetViewAngle(fy);
}

/* Converts a RenderEngineVtk ImageType to a RenderClient RenderImageType.
 NOTE: If RenderImageType expands to have more image types, e.g., two kinds of
 depth images supported, this logic will have to be revisited. */
RenderImageType VtkToRenderImageType(ImageType image_type) {
  switch (image_type) {
    case ImageType::kColor:
      return RenderImageType::kColorRgba8U;
    case ImageType::kDepth:
      return RenderImageType::kDepthDepth32F;
    case ImageType::kLabel:
      return RenderImageType::kLabel16I;
  }
  DRAKE_UNREACHABLE();
}

std::string_view ImageTypeToString(ImageType image_type) {
  switch (image_type) {
    case ImageType::kColor:
      return "color";
    case ImageType::kDepth:
      return "depth";
    case ImageType::kLabel:
      return "label";
  }
  DRAKE_UNREACHABLE();
}

void LogFrameStart(ImageType image_type, int64_t scene_id) {
  log()->debug("RenderEngineGltfClient: rendering {} scene id {}.",
               ImageTypeToString(image_type), scene_id);
}

void LogFrameGltfExportPath(ImageType image_type, const std::string& path) {
  log()->debug("RenderEngineGltfClient: {} scene exported to '{}'.",
               ImageTypeToString(image_type), path);
}

void LogFrameServerResponsePath(ImageType image_type, const std::string& path) {
  log()->debug(
      "RenderEngineGltfClient: {} server response image saved to '{}'.",
      ImageTypeToString(image_type), path);
}

/* Deletes the files, i.e., `scene_path` and `image_path`, and logs if verbose.
 This function is called only when get_params().cleanup == true. */
void CleanupFrame(const std::string& scene_path, const std::string& image_path,
                  bool verbose) {
  fs::remove(scene_path);
  fs::remove(image_path);
  if (verbose) {
    log()->debug("RenderEngineGltfClient: deleted unused files {} and {}.",
                 scene_path, image_path);
  }
}

/* Returns a formatted scene file name, e.g., 0000000000000000XYZ-color.gltf.
 NOTE: The maximum number of digits in an int64_t is 19. */
std::string GetSceneFileName(ImageType image_type, int64_t scene_id) {
  return fmt::format("{:0>19}-{}.gltf", scene_id,
                     ImageTypeToString(image_type));
}

/* Searches the gltf and identifies all root nodes (a node that doesn't appear
 in a "children" list). Returns a map from the *index* of each root node to that
 node's transform *in the file* (T_FN). The transform can consist of rotation,
 translation, *and* scale components. */
std::map<int, Matrix4<double>> FindRootNodes(const nlohmann::json& gltf) {
  std::map<int, Matrix4<double>> roots;
  if (gltf.contains("nodes")) {
    std::set<int> indices;
    // Cull children.
    const nlohmann::json& nodes = gltf["nodes"];
    const int node_count = ssize(nodes);
    for (int i = 0; i < node_count; ++i) indices.insert(indices.end(), i);
    for (const nlohmann::json& node : nodes) {
      if (node.contains("children")) {
        for (const auto& c : node["children"]) {
          const int c_index = c.get<int>();
          indices.erase(c_index);
        }
      }
    }
    // TODO(SeanCurtis-TRI): We could test to make sure that the root node is
    // actually referenced in a scene (or even the default scene) and omit it
    // from the set if it's not.

    // Compute transforms for the identified root nodes.
    for (int n : indices) {
      const auto& node = nodes[n];
      Matrix4<double> T_FN;
      if (node.contains("matrix")) {
        T_FN = EigenMatrixFromGltfMatrix(node["matrix"]);
      } else {
        T_FN = Matrix4<double>::Identity();
        if (node.contains("translation")) {
          const auto& t = node["translation"];
          const Vector3<double> p_GN(t[0].get<double>(), t[1].get<double>(),
                                     t[2].get<double>());
          T_FN.block<3, 1>(0, 3) = p_GN.transpose();
        }
        if (node.contains("rotation")) {
          const auto& q = node["rotation"];
          // glTF rotation is (x, y, z, w) quaternion. Eigen is (w, x, y, z).
          const Quaternion<double> quat_GN(
              q[3].get<double>(), q[0].get<double>(), q[1].get<double>(),
              q[2].get<double>());
          T_FN.block<3, 3>(0, 0) = math::RotationMatrixd(quat_GN).matrix();
        }
        if (node.contains("scale")) {
          const auto& s = node["scale"];
          for (int i = 0; i < 3; ++i) {
            T_FN.block<3, 1>(0, i) *= s[i].get<double>();
          }
        }
      }
      roots.insert(roots.end(), {n, T_FN});
    }
  }

  return roots;
}

/* Effectively sets the world pose of the single "geometry" represented by one
 or more nodes defined in a glTF file. It achieves this by setting the
 transforms on the `root_nodes` of the glTF and *only* the root nodes. All other
 nodes should maintain their fixed poses *relative* to their root nodes. It's
 important that `root_nodes` contain *only* root nodes, such as those found by
 FindRootNodes().

 @param gltf         The json representing the gltf file; its contents will be
                     modified.
 @param root_nodes   The set of nodes to set (indicated by integers in the range
                     [0, N-1] such that the gltf contains N nodes.
 @param X_WG         The pose of the Drake geometry in the world frame.
 @param scale        The scale to apply to all the indicated nodes (i.e., S_WF).
 @param strip        If true, removes any translation, rotation, or scale
                     properties from the targeted nodes. */
void SetRootPoses(nlohmann::json* gltf,
                  const std::map<int, Matrix4<double>>& root_nodes,
                  const math::RigidTransformd& X_WG,
                  const Vector3<double>& scale, bool strip) {
  /* We have to do some extra work for poses. A *correct* glTF file is defined
   in the file's frame F as y up. Drake poses things in a geometry frame G which
   is z up. When VTK exports a glTF file, it doesn't rotate from F to G (y up
   to z up). We can't simply apply the geometry pose X_WG to the glTF's data. We
   need to introduce X_GF, the transform necessary to take the data from the
   y-up glTF frame to the z-up Drake frame.

   Furthermore, the transform in glTF is not a Drake RigidTransform; it is
   an affine transform. glTF documents the transform T = X_t * X_r * S, where
   X_t and X_r are RigidTransforms consisting of a translation and rotation,
   respectively. S is a (possibly non-uniform) scale transform.

   Therefore, the transform we need to apply to indicated nodes includes: the
   geometry pose X_WG, the geometry scale transform determined by scale (S_WG),
   the y-up-to-z-up transform (X_GF), and the node's transform within its file,
   T_FN. Or, more succinctly:

      T_WN = X_WG * S_WG * X_GF * T_FN, or
      T_WN =               T_WF * T_FN.
   */

  // T_WF isn't truly T_WF at construction. We'll construct it piecemeal as
  // we concatenate transforms on its right side as indicated above.
  Matrix4<double> T_WF = X_WG.GetAsMatrix4();
  // S_WG is `scale` on the diagonal. Multiplication with X_WG has the effect of
  // scaling the corresponding columns.
  for (int i = 0; i < 3; ++i) {
    T_WF.block<3, 1>(0, i) *= scale(i);
  }
  const math::RigidTransformd X_GF(
      math::RotationMatrixd::MakeXRotation(M_PI / 2));
  T_WF *= X_GF.GetAsMatrix4();

  if (gltf->contains("nodes")) {
    nlohmann::json& nodes = (*gltf)["nodes"];
    const int node_count = ssize(nodes);
    for (const auto& [n, T_FN] : root_nodes) {
      DRAKE_DEMAND(n >= 0 && n < node_count);
      if (strip) {
        nodes[n].erase("translation");
        nodes[n].erase("position");
        nodes[n].erase("scale");
      }
      nodes[n]["matrix"] = GltfMatrixFromEigenMatrix(T_WF * T_FN);
    }
  }
}

// TODO(SeanCurtis-TRI): This is a vague hack. It provides the right label
// colors, but it leaves a great deal of cruft in the gltf: samplers, textures,
// images, and, most importantly, data in the buffer. Ideally, label and depth
// renders shouldn't transmit any of this otherwise useless data.

/* Changes all material definitions to be an emissive flat color. This removes
 all references to textures. */
void ChangeToLabelMaterials(nlohmann::json* gltf, const Rgba& color) {
  if (gltf->contains("materials")) {
    auto& materials = (*gltf)["materials"];
    for (auto& mat : materials) {
      // We're keeping the other material values (e.g., doubleSided, name,
      // extensions, extras, etc.)
      mat.erase("normalTexture");
      mat.erase("occlusionTexture");
      mat.erase("emissiveTexture");
      mat["emissiveFactor"] = {color.r(), color.g(), color.b()};
      auto& pbr = mat["pbrMetallicRoughness"];
      pbr["baseColorFactor"] = {color.r(), color.g(), color.b(), 1.0};
      pbr.erase("baseColorTexture");
      pbr.erase("metallicFactor");
      pbr.erase("roughnessFactor");
      pbr.erase("metallicRoughnessTexture");
    }
  }
}

}  // namespace

RenderEngineGltfClient::RenderEngineGltfClient(
    const RenderEngineGltfClientParams& parameters)
    : render_client_{std::make_unique<RenderClient>(parameters)} {}

RenderEngineGltfClient::RenderEngineGltfClient(
    const RenderEngineGltfClient& other)
    : RenderEngineVtk(other),
      render_client_(std::make_unique<RenderClient>(other.get_params())),
      gltfs_(other.gltfs_) {}

RenderEngineGltfClient::~RenderEngineGltfClient() = default;

std::unique_ptr<RenderEngine> RenderEngineGltfClient::DoClone() const {
  return std::unique_ptr<RenderEngineGltfClient>(
      new RenderEngineGltfClient(*this));
}

void RenderEngineGltfClient::DoRenderColorImage(
    const ColorRenderCamera& camera, ImageRgba8U* color_image_out) const {
  const int64_t color_scene_id = GetNextSceneId();
  if (get_params().verbose) {
    LogFrameStart(ImageType::kColor, color_scene_id);
  }

  const RenderingPipeline& color_pipeline =
      get_mutable_pipeline(ImageType::kColor);
  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera.core(), camera.show_window(), color_pipeline,
               "Color Image");
  PerformVtkUpdate(color_pipeline);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(camera.core(),
                           color_pipeline.renderer->GetActiveCamera());
  const std::string scene_path =
      fs::path(temp_directory()) /
      GetSceneFileName(ImageType::kColor, color_scene_id);
  ExportScene(scene_path, ImageType::kColor);
  if (get_params().verbose) {
    LogFrameGltfExportPath(ImageType::kColor, scene_path);
  }

  const std::string image_path = render_client_->RenderOnServer(
      camera.core(), VtkToRenderImageType(ImageType::kColor), scene_path,
      MimeType());
  if (get_params().verbose) {
    LogFrameServerResponsePath(ImageType::kColor, image_path);
  }

  // Load the returned image back to the drake buffer.
  render_client_->LoadColorImage(image_path, color_image_out);
  if (get_params().cleanup) {
    CleanupFrame(scene_path, image_path, get_params().verbose);
  }
}

void RenderEngineGltfClient::DoRenderDepthImage(
    const DepthRenderCamera& camera, ImageDepth32F* depth_image_out) const {
  const int64_t depth_scene_id = GetNextSceneId();

  if (get_params().verbose) {
    LogFrameStart(ImageType::kDepth, depth_scene_id);
  }

  const RenderingPipeline& depth_pipeline =
      get_mutable_pipeline(ImageType::kDepth);
  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera, depth_pipeline);
  PerformVtkUpdate(depth_pipeline);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(camera.core(),
                           depth_pipeline.renderer->GetActiveCamera());
  const std::string scene_path =
      fs::path(temp_directory()) /
      GetSceneFileName(ImageType::kDepth, depth_scene_id);
  ExportScene(scene_path, ImageType::kDepth);
  if (get_params().verbose) {
    LogFrameGltfExportPath(ImageType::kDepth, scene_path);
  }

  const std::string image_path = render_client_->RenderOnServer(
      camera.core(), VtkToRenderImageType(ImageType::kDepth), scene_path,
      MimeType(), camera.depth_range());
  if (get_params().verbose) {
    LogFrameServerResponsePath(ImageType::kDepth, image_path);
  }

  // Load the returned image back to the drake buffer.
  render_client_->LoadDepthImage(image_path, depth_image_out);
  if (get_params().cleanup) {
    CleanupFrame(scene_path, image_path, get_params().verbose);
  }
}

void RenderEngineGltfClient::DoRenderLabelImage(
    const ColorRenderCamera& camera, ImageLabel16I* label_image_out) const {
  const int64_t label_scene_id = GetNextSceneId();

  if (get_params().verbose) {
    LogFrameStart(ImageType::kLabel, label_scene_id);
  }

  const RenderingPipeline& label_pipeline =
      get_mutable_pipeline(ImageType::kLabel);
  // Update the VTK scene before exporting to glTF.
  UpdateWindow(camera.core(), camera.show_window(), label_pipeline,
               "Label Image");
  PerformVtkUpdate(label_pipeline);

  // Export and render the glTF scene.
  SetGltfCameraPerspective(camera.core(),
                           label_pipeline.renderer->GetActiveCamera());
  const std::string scene_path =
      fs::path(temp_directory()) /
      GetSceneFileName(ImageType::kLabel, label_scene_id);
  ExportScene(scene_path, ImageType::kLabel);
  if (get_params().verbose) {
    LogFrameGltfExportPath(ImageType::kLabel, scene_path);
  }

  const std::string image_path = render_client_->RenderOnServer(
      camera.core(), VtkToRenderImageType(ImageType::kLabel), scene_path,
      MimeType());
  if (get_params().verbose) {
    LogFrameServerResponsePath(ImageType::kLabel, image_path);
  }

  // Load the returned image back to the drake buffer.
  /* NOTE: The loaded image from `image_path` is expected to be a colored label
   image that will then be converted to an actual label image.  The server has
   no knowledge of the conversion formula, and thus, a colored label image is
   returned. */
  const int width = label_image_out->width();
  const int height = label_image_out->height();
  ImageRgba8U colored_label_image(width, height);
  render_client_->LoadColorImage(image_path, &colored_label_image);

  // By convention (see render_gltf_client_doxygen.h), server-only artifacts are
  // colored white to indicate the "don't care" semantic.
  // Convert from RGB to Label.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const uint8_t r = colored_label_image.at(x, y)[0];
      const uint8_t g = colored_label_image.at(x, y)[1];
      const uint8_t b = colored_label_image.at(x, y)[2];
      label_image_out->at(x, y)[0] =
          (r == 255 && g == 255 && b == 255)
              ? render::RenderLabel::kDontCare
              : RenderEngine::MakeLabelFromRgb(r, g, b);
    }
  }

  if (get_params().cleanup) {
    CleanupFrame(scene_path, image_path, get_params().verbose);
  }
}

std::string RenderEngineGltfClient::DoGetParameterYaml() const {
  return yaml::SaveYamlString(get_params(), "RenderEngineGltfClientParams");
}

void RenderEngineGltfClient::ExportScene(const std::string& export_path,
                                         ImageType image_type) const {
  // TODO(SeanCurtis-TRI): Given the ability to edit the gltf in place, we
  //  should use VTK to create the gltf and merge the gltfs *once* and use that
  //  as the reference gltf. Then, with a mapping from geometry id to node
  //  index, we can simply mutate the poses before saving the result. This
  //  would decrease the cost of preparing the gltf file for transmission
  //  *significantly*.  It would probably be good to key this on scene graphs
  //  perception version so I can rebuild the VTK as appropriate.
  vtkNew<vtkGLTFExporter> gltf_exporter;
  gltf_exporter->InlineDataOn();
  gltf_exporter->SetRenderWindow(get_mutable_pipeline(image_type).window);
  const std::string gltf_contents = gltf_exporter->WriteToString();
  nlohmann::json gltf = nlohmann::json::parse(gltf_contents);

  // Merge in gltf files. The path below is an imaginary path that should not
  // appear in any error messages -- unless we start using extensions/extras in
  // RenderEngineVtk and VTK starts exporting those values into a glTF file.
  MergeRecord merge_record("scene_graph.gltf");
  for (const auto& [id, record] : gltfs_) {
    nlohmann::json temp = record.contents;
    if (image_type == render_vtk::internal::kLabel) {
      const Rgba color = RenderEngine::MakeRgbFromLabel(record.label);
      ChangeToLabelMaterials(&temp, color);
    }
    MergeGltf(&gltf, std::move(temp), record.name, &merge_record);
  }

  // TODO(SeanCurtis-TRI): Update materials for label images. Because the gltf
  // might contain arbitrary materials, it might make more sense to do this
  // as part of registration, storing two versions of the json (the color/depth
  // and the label). It might also make sense to strip out the textures if
  // doing depth as unnecessary. A future optimization.

  std::ofstream f(export_path);
  f << gltf;
  f.close();
  // If there were errors in writing, the state of the stream is guaranteed to
  // be up to date after closing it. Now we can check for errors.
  if (!f) {
    throw std::runtime_error(
        "RenderEngineGltfClient: Error writing exported scene data to disk.");
  }
}

void RenderEngineGltfClient::DoUpdateVisualPose(
    GeometryId id, const math::RigidTransformd& X_WG) {
  auto iter = gltfs_.find(id);
  if (iter != gltfs_.end()) {
    GltfRecord& gltf = iter->second;
    SetRootPoses(&gltf.contents, gltf.root_nodes, X_WG, gltf.scale,
                 false /* strip */);
    return;
  }
  RenderEngineVtk::DoUpdateVisualPose(id, X_WG);
}

bool RenderEngineGltfClient::DoRemoveGeometry(GeometryId id) {
  if (gltfs_.contains(id)) {
    gltfs_.erase(id);
    return true;
  } else {
    return RenderEngineVtk::DoRemoveGeometry(id);
  }
}

void RenderEngineGltfClient::ImplementGeometry(const Mesh& mesh,
                                               void* user_data) {
  auto& data = *static_cast<RegistrationData*>(user_data);
  const std::string extension = mesh.extension();
  if (extension == ".obj") {
    // This invokes RenderEngineVtk::ImplementObj().
    data.accepted = ImplementObj(mesh, data);
  } else if (extension == ".gltf") {
    data.accepted = ImplementGltf(mesh, data);
  } else {
    static const logging::Warn one_time(
        "RenderEngineGltfClient only supports Mesh specifications which use "
        ".obj or .gltf files. Mesh specifications using other mesh types "
        "(e.g., .stl, .dae, etc.) will be ignored.");
    data.accepted = false;
  }
}

namespace {

// If `item_inout` has a field named `uri` and it is not a `data:` URI, replaces
// the field's value with a base64-encoded `data:` URI.
//
// In glTF 2.0, URIs can only appear in two places:
//  "images": [ { "uri": "some.png" } ]
//  "buffers": [ { "uri": "some.bin", "byteLength": 1024 } ]
//
// As documented on MergeGltf(), this is how RenderEngineGltfClient converts
// external resources to embedded ata URIs.
void MaybeEmbedDataUri(nlohmann::json* item_inout,
                       const MeshSource& mesh_source,
                       std::string_view array_name) {
  DRAKE_DEMAND(item_inout != nullptr);
  nlohmann::json& item = *item_inout;
  if (!item.contains("uri")) {
    return;
  }
  const std::string_view uri = item["uri"].template get<std::string_view>();
  if (uri.substr(0, 5) == "data:") {
    return;
  }
  std::string content;
  if (mesh_source.is_path()) {
    content = ReadFileOrThrow(mesh_source.path().parent_path() / uri);
  } else {
    DRAKE_DEMAND(mesh_source.is_in_memory());
    const auto file_source_iter =
        mesh_source.in_memory().supporting_files.find(uri);
    if (file_source_iter == mesh_source.in_memory().supporting_files.end()) {
      throw std::runtime_error(fmt::format(
          "RenderEngineGltfClient cannot add an in-memory Mesh. The Mesh's "
          "glTF ('{}') file names a uri ('{}') for {} that is not contained "
          "within the supporting files.",
          mesh_source.in_memory().mesh_file.filename_hint(), uri, array_name));
    }
    content = std::visit<std::string>(overloaded{[](const fs::path& path) {
                                                   return ReadFileOrThrow(path);
                                                 },
                                                 [](const MemoryFile& file) {
                                                   return file.contents();
                                                 }},
                                      file_source_iter->second);
  }

  // Note: content may still be empty; we'll defer to the server to handle it.

  item["uri"] =
      fmt::format("data:application/octet-stream;base64,{}",
                  common_robotics_utilities::base64_helpers::Encode(
                      std::vector<uint8_t>(content.begin(), content.end())));
}

void EmbedFileUris(nlohmann::json* gltf_ptr, const MeshSource& mesh_source) {
  nlohmann::json& gltf = *gltf_ptr;
  // Iterate through buffers and images.
  for (std::string_view array_name : {"buffers", "images"}) {
    auto& array = gltf[array_name];
    for (size_t i = 0; i < array.size(); ++i) {
      MaybeEmbedDataUri(&array[i], mesh_source, array_name);
    }
  }
}

}  // namespace

bool RenderEngineGltfClient::ImplementGltf(
    const Mesh& mesh, const RenderEngineVtk::RegistrationData& data) {
  nlohmann::json mesh_data = ReadJsonFile(mesh.source());

  // We'll end up merging this glTF into the VTK-generated glTF and broadcasting
  // it over a wire. The idea of "file-relative" URIs becomes meaningless at
  // that point. So, we'll simply convert all file URIs to data URIs.
  EmbedFileUris(&mesh_data, mesh.source());

  // TODO(SeanCurtis-TRI) What to do about a gltf that has no materials? We need
  // to apply the same logic of the data.properties as we do to OBJ. We'll
  // defer for now because the expectation is that the gltf is used *because*
  // of materials.

  std::map<int, Matrix4<double>> root_nodes = FindRootNodes(mesh_data);
  SetRootPoses(&mesh_data, root_nodes, data.X_WG, mesh.scale3(), true);

  DRAKE_DEMAND(!gltfs_.contains(data.id));
  const MeshSource& mesh_source = mesh.source();
  const std::string gltf_name = mesh_source.description();
  gltfs_.insert({data.id,
                 {gltf_name, std::move(mesh_data), std::move(root_nodes),
                  mesh.scale3(), GetRenderLabelOrThrow(data.properties)}});
  return true;
}

Eigen::Matrix4d RenderEngineGltfClient::CameraModelViewTransformMatrix(
    ImageType image_type) const {
  vtkCamera* cam = get_mutable_pipeline(image_type).renderer->GetActiveCamera();
  const vtkMatrix4x4* vtk_mat = cam->GetModelViewTransformMatrix();
  Eigen::Matrix4d ret;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ret(i, j) = vtk_mat->GetElement(i, j);
    }
  }
  return ret;
}

void RenderEngineGltfClient::SetHttpService(
    std::unique_ptr<HttpService> service) {
  render_client_->SetHttpService(std::move(service));
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

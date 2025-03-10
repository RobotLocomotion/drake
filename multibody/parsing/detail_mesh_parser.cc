#include "drake/multibody/parsing/detail_mesh_parser.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <istream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/mesh_source.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/multibody/parsing/detail_make_model_name.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::geometry::TriangleSurfaceMesh;
using drake::internal::DiagnosticPolicy;

// The parse result of attempting to read a single object from OBJ data.
struct NamedMesh {
  std::string name;
  std::unique_ptr<TriangleSurfaceMesh<double>> mesh;
};

// Callback function for capturing the parsed object name; required by tinyobj's
// API. `storage` is provided by the caller.
void CaptureName(void* storage, const char* parsed_name) {
  *static_cast<std::string*>(storage) = parsed_name;
}

// Attempts to parse a single .obj mesh, returning the resulting triangle mesh
// and its name (if any is defined). If there is a parsing failure, the return
// value contains a null mesh.
NamedMesh DoGetObjMesh(const DiagnosticPolicy& diagnostic,
                       const geometry::MeshSource& obj_source) {
  const Eigen::Vector3d kUnitScale(1, 1, 1);
  std::optional<TriangleSurfaceMesh<double>> mesh =
      geometry::internal::DoReadObjToSurfaceMesh(obj_source, kUnitScale,
                                                 diagnostic);

  if (!mesh.has_value()) {
    return {};
  }

  // A mesh exists, let's see if it had a name. We'll parse the obj again,
  // flagging object names ("o some_name"). The last object name encountered
  // will be the de facto object name.
  std::string object_name;
  tinyobj::callback_t callback;
  callback.object_cb = CaptureName;

  std::unique_ptr<std::istream> input_stream;
  if (obj_source.is_path()) {
    input_stream = std::make_unique<std::ifstream>(obj_source.path());
  } else {
    DRAKE_DEMAND(obj_source.is_in_memory());
    // Note: This is copying the in-memory data. We can avoid this by creating
    // a custom buffer type, but it suffices for now. In practice, we don't
    // expect this to be exercised, because there's no simple mechanism for
    // exercising this function with an in-memory mesh source.
    input_stream = std::make_unique<std::istringstream>(
        obj_source.in_memory().mesh_file.contents());
  }

  const bool result =
      tinyobj::LoadObjWithCallback(*input_stream, callback, &object_name);
  // If we have a mesh, then no error should be encountered in attempting to
  // find the name.
  DRAKE_DEMAND(result);
  // Note: the name may still be *empty*. This simply means the obj didn't
  // define an object name.
  return {object_name,
          std::make_unique<TriangleSurfaceMesh<double>>(std::move(*mesh))};
}

// Returns the mesh from the named file. As the structure of NamedMesh implies,
// this is geometry only -- no materials.
//
// N.B. This layer of abstraction leaves the door open for implementing
// GetMeshFromString() once we can support handling SceneGraph geometry from
// an in-memory source.
NamedMesh GetMeshFromFile(const DiagnosticPolicy& diagnostic,
                          const std::filesystem::path& filename) {
  {
    // Quick probe of the file so that, if there's a problem, it can be properly
    // registered with the diagnostic policy.
    std::ifstream input_stream(filename);
    if (!input_stream.is_open()) {
      diagnostic.Error(fmt::format("Cannot open file '{}'", filename.string()));
      return {};
    }
  }

  std::string ext = filename.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (ext != ".obj") {
    diagnostic.Error(fmt::format(
        "Adding models from meshes only works with .obj files. Given '{}'.",
        filename.string()));
  }

  return DoGetObjMesh(diagnostic, filename);
}

}  // namespace

std::optional<ModelInstanceIndex> AddModelFromMesh(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  // TODO(SeanCurtis-TRI): We can now parse from in-memory obj specification.
  // However, DataSource doesn't give us any ability to validate the *type* of
  // the file contents provided. It would be nice if DataSource plumbed the
  // user-provided extension through to here.
  if (!data_source.IsFilename()) {
    throw std::runtime_error(
        "Parser does not yet support adding models using in-memory OBJ data. "
        "The OBJ data must be referenced as a file.");
  }
  DRAKE_THROW_UNLESS(!workspace.plant->is_finalized());

  const std::string filename = data_source.GetAbsolutePath();

  NamedMesh named_mesh = GetMeshFromFile(workspace.diagnostic, filename);

  if (named_mesh.mesh == nullptr) {
    // Reasons for there being no mesh should already have been dispatched to
    // the diagnostic policy.
    return std::nullopt;
  }

  // Resolve the naming protocol - see Parser documentation.
  std::string candidate_name = model_name;
  if (candidate_name.empty()) {
    candidate_name = named_mesh.name;
  }
  if (candidate_name.empty()) {
    candidate_name = std::filesystem::path(filename).stem();
  }

  // Register model instance, body, and collision and visual geometries.
  MultibodyPlant<double>& plant = *workspace.plant;
  std::string model_instance_name =
      MakeModelName(candidate_name, parent_model_name, workspace);
  const ModelInstanceIndex model_instance =
      plant.AddModelInstance(model_instance_name);

  const CalcSpatialInertiaResult result = CalcSpatialInertiaImpl(
      *named_mesh.mesh, 1000.0 /* water density ≈ 1000 kg/m³ */);
  if (std::holds_alternative<std::string>(result)) {
    workspace.diagnostic.Error(std::get<std::string>(result));
    return {};
  }
  const auto M_BBo_B = std::get<SpatialInertia<double>>(result);
  const RigidBody<double>& body =
      plant.AddRigidBody(candidate_name, model_instance, M_BBo_B);

  if (plant.geometry_source_is_registered()) {
    // TODO(SeanCurtis-TRI): Consider using named_mesh.name as part of the
    // geometry names.
    // TODO(SeanCurtis-TRI): It would be better to use a single geometry and
    // assign all three roles to it. This would require access to the SceneGraph
    // and some shenanigans to placate MbP.
    const auto X_BG = math::RigidTransformd::Identity();
    const geometry::Mesh mesh(filename);
    // We don't know any better than providing an empty ProximityProperties
    // and waiting for SceneGraph to backfill with the default values.
    plant.RegisterCollisionGeometry(body, X_BG, mesh, "collision",
                                    geometry::ProximityProperties());
    // TODO(SeanCurtis-TRI): If there's a material applied to the object, use
    // the specified color.
    plant.RegisterVisualGeometry(body, X_BG, mesh, "visual");
  }

  return model_instance;
}

MeshParserWrapper::MeshParserWrapper() = default;
MeshParserWrapper::~MeshParserWrapper() = default;

std::optional<ModelInstanceIndex> MeshParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddModelFromMesh(data_source, model_name, parent_model_name,
                          workspace);
}

std::vector<ModelInstanceIndex> MeshParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  std::optional<ModelInstanceIndex> maybe_index =
      AddModelFromMesh(data_source, "", parent_model_name, workspace);
  if (maybe_index.has_value()) {
    return {*maybe_index};
  }
  return {};
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

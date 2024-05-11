#include "drake/multibody/parsing/detail_usd_parser.h"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/sphere.h"
#include <fmt/format.h>

#include "drake/common/find_runfiles.h"
#include "drake/common/unused.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_usd_geometry.h"

namespace drake {
namespace multibody {
namespace internal {

namespace fs = std::filesystem;

struct UsdStageMetadata {
    double meters_per_unit = 1.0;
    pxr::TfToken up_axis = pxr::TfToken("Z");
};

class UsdParser {
 public:
  explicit UsdParser(const ParsingWorkspace& ws);
  ~UsdParser() = default;
  std::vector<ModelInstanceIndex> AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name);

 private:
  void ProcessPrim(const pxr::UsdPrim& prim);
  void ProcessRigidBody(const pxr::UsdPrim& prim, bool is_static);
  UsdStageMetadata GetStageMetadata(const pxr::UsdStageRefPtr stage);
  std::unique_ptr<geometry::Shape> CreateCollisionGeometry(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateVisualGeometry(
    const pxr::UsdPrim& prim);
  const RigidBody<double>& CreateRigidBody(const pxr::UsdPrim& prim);

  inline static std::vector<std::string> mesh_filenames_;
  const ParsingWorkspace& w_;
  UsdStageMetadata metadata_;
};

namespace {
void InitializeOpenUsdLibrary() {
  // Register all relevant plugins.
  auto& registry = pxr::PlugRegistry::PlugRegistry::GetInstance();
  std::vector<std::string> json_paths{{
      "openusd_internal/pxr/usd/ar/plugInfo.json",
      "openusd_internal/pxr/usd/ndr/plugInfo.json",
      "openusd_internal/pxr/usd/sdf/plugInfo.json",
      "openusd_internal/pxr/usd/usd/plugInfo.json",
      "openusd_internal/pxr/usd/usdGeom/plugInfo.json",
      "openusd_internal/pxr/usd/usdPhysics/plugInfo.json",
      "openusd_internal/pxr/usd/usdShade/plugInfo.json",
  }};
  for (const auto& json_path : json_paths) {
    const RlocationOrError location = FindRunfile(json_path);
    if (!location.error.empty()) {
      throw std::runtime_error(location.error);
    }
    const fs::path info_dir = fs::path(location.abspath).parent_path();
    registry.RegisterPlugins(info_dir.string());
  }
}
}  // namespace

UsdParserWrapper::UsdParserWrapper() {
  static const int ignored = []() {
    InitializeOpenUsdLibrary();
    return 0;
  }();
  unused(ignored);
}

UsdParserWrapper::~UsdParserWrapper() = default;

std::optional<ModelInstanceIndex> UsdParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  unused(data_source, model_name, parent_model_name, workspace);
  throw std::runtime_error("UsdParser::AddModel is not implemented");
}

std::vector<ModelInstanceIndex> UsdParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  UsdParser parser(workspace);
  return parser.AddAllModels(data_source, parent_model_name);
}

UsdParser::UsdParser(const ParsingWorkspace& ws) : w_{ws} { }

std::unique_ptr<geometry::Shape> UsdParser::CreateVisualGeometry(
  const pxr::UsdPrim& prim) {
  double mpu = metadata_.meters_per_unit;
  if (prim.IsA<pxr::UsdGeomCube>()) {
    return CreateGeometryBox(prim, mpu, w_);
  } else if (prim.IsA<pxr::UsdGeomSphere>()) {
    return CreateGeometryEllipsoid(prim, mpu, w_);
  } else if (prim.IsA<pxr::UsdGeomCapsule>()) {
    return CreateGeometryCapsule(prim, mpu, metadata_.up_axis, w_);
  } else if (prim.IsA<pxr::UsdGeomCylinder>()) {
    return CreateGeometryCylinder(prim, mpu, metadata_.up_axis, w_);
  } else if (prim.IsA<pxr::UsdGeomMesh>()) {
    // Create an obj file for each mesh and pass the filename into
    // the constructor of geometry::Mesh. This is a temporary solution while
    // https://github.com/RobotLocomotion/drake/issues/15263 is being worked on
    std::string obj_filename = fmt::format("{}.obj", mesh_filenames_.size());
    mesh_filenames_.push_back(obj_filename);
    return CreateGeometryMesh(obj_filename, prim, mpu, w_);
  } else {
    pxr::TfToken prim_type = prim.GetTypeName();
    w_.diagnostic.Error(fmt::format("Unsupported Prim type: {}", prim_type));
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> UsdParser::CreateCollisionGeometry(
  const pxr::UsdPrim& prim) {
  // For now, we use the raw visual geometry for collision detection
  // for all geometry types
  return CreateVisualGeometry(prim);
}

const RigidBody<double>& UsdParser::CreateRigidBody(const pxr::UsdPrim& prim) {
  double mpu = metadata_.meters_per_unit;
  SpatialInertia<double> si;
  if (prim.IsA<pxr::UsdGeomCube>()) {
    si = CreateSpatialInertiaForBox(prim, mpu, w_);
  } else if (prim.IsA<pxr::UsdGeomSphere>()) {
    si = CreateSpatialInertiaForEllipsoid(prim, mpu, w_);
  } else if (prim.IsA<pxr::UsdGeomCapsule>()) {
    si = CreateSpatialInertiaForCapsule(prim, mpu, metadata_.up_axis, w_);
  } else if (prim.IsA<pxr::UsdGeomCylinder>()) {
    si = CreateSpatialInertiaForCylinder(prim, mpu, metadata_.up_axis, w_);
  } else if (prim.IsA<pxr::UsdGeomMesh>()) {
    // TODO(hong-nvidia): Determine how to create SpatialInertia for a mesh
    si = SpatialInertia<double>::MakeUnitary();
  } else {
    si.SetNaN();
    pxr::TfToken prim_type = prim.GetTypeName();
    w_.diagnostic.Error(fmt::format("Unsupported Prim type: {}", prim_type));
  }
  return w_.plant->AddRigidBody(
    fmt::format("{}-RigidBody", prim.GetPath().GetString()), si);
}

void UsdParser::ProcessRigidBody(const pxr::UsdPrim& prim,
  bool is_static) {
  auto collision_geometry = CreateCollisionGeometry(prim);
  auto visual_geometry = CreateVisualGeometry(prim);
  if (!collision_geometry || !visual_geometry) {
    w_.diagnostic.Error(fmt::format("Failed to create collision or visual "
      "geometry for prim at {}", prim.GetPath().GetString()));
    return;
  }

  const RigidBody<double>* rigid_body;
  math::RigidTransform<double> transform_relative_to_rigid_body;
  math::RigidTransform<double> prim_transform =
    GetPrimRigidTransform(prim, metadata_.meters_per_unit);
  if (is_static) {
    rigid_body = &w_.plant->world_body();
    transform_relative_to_rigid_body = prim_transform;
  } else {
    rigid_body = &CreateRigidBody(prim);
    transform_relative_to_rigid_body =
      math::RigidTransform<double>::Identity();
    w_.plant->SetDefaultFreeBodyPose(*rigid_body, prim_transform);
  }

  w_.plant->RegisterCollisionGeometry(
    *rigid_body,
    transform_relative_to_rigid_body,
    *collision_geometry,
    fmt::format("{}-CollisionGeometry", prim.GetPath().GetString()),
    GetPrimFriction(prim));

  w_.plant->RegisterVisualGeometry(
    *rigid_body,
    transform_relative_to_rigid_body,
    *visual_geometry,
    fmt::format("{}-VisualGeometry", prim.GetPath().GetString()),
    GetGeomPrimColor(prim));
}

void UsdParser::ProcessPrim(const pxr::UsdPrim& prim) {
  drake::log()->info(fmt::format("Processing {}", prim.GetPath().GetString()));
  if (prim.HasAPI(pxr::TfToken("PhysicsCollisionAPI"))) {
    if (prim.HasAPI(pxr::TfToken("PhysicsRigidBodyAPI"))) {
      ProcessRigidBody(prim, false);
    } else {
      ProcessRigidBody(prim, true);
    }
  }
}

UsdStageMetadata UsdParser::GetStageMetadata(const pxr::UsdStageRefPtr stage) {
  UsdStageMetadata metadata;
  if (!stage->GetMetadata(pxr::TfToken("metersPerUnit"),
    &metadata.meters_per_unit)) {
    w_.diagnostic.Warning(fmt::format(
      "Failed to read metersPerUnit in stage metadata. "
      "Using the default value ({}) instead.", metadata.meters_per_unit));
  }
  if (!stage->GetMetadata(pxr::TfToken("upAxis"), &metadata.up_axis)) {
    w_.diagnostic.Warning(fmt::format(
      "Failed to read upAxis in stage metadata. "
      "Using the default value ({}) instead.", metadata.up_axis));
  }
  if (metadata.up_axis != "Z") {
    throw std::runtime_error("Only Z-up stages are supported at the moment");
  }
  return metadata;
}

std::vector<ModelInstanceIndex> UsdParser::AddAllModels(
  const DataSource& data_source,
  const std::optional<std::string>& parent_model_name) {
  unused(parent_model_name);

  if (data_source.IsContents()) {
    throw std::runtime_error(
      "Ingesting raw USD content from DataSource is not implemented");
  }

  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(
    data_source.GetAbsolutePath());
  if (!stage) {
    w_.diagnostic.Error(fmt::format("Failed to open USD stage",
      data_source.filename()));
  }

  metadata_ = GetStageMetadata(stage);

  for (pxr::UsdPrim prim : stage->Traverse()) {
    ProcessPrim(prim);
  }

  // Returning an empty vector, since only rigidbody colliders are supported
  // at the moment (i.e., no actual models are involved).
  return std::vector<ModelInstanceIndex>();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

#include "drake/multibody/parsing/detail_usd_parser.h"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "pxr/base/gf/transform.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/xformable.h"
#include <fmt/format.h>

#include "drake/common/find_runfiles.h"
#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/detail_common.h"

namespace drake {
namespace multibody {
namespace internal {

namespace fs = std::filesystem;
#if 0
namespace pxr = drake_vendor_pxr;
#endif

struct UsdStageMetadata {
    double meters_per_unit = 1.0;
    pxr::TfToken up_axis = pxr::TfToken("Z");
};

namespace {
void InitializeOpenUsdLibrary() {
#if 0
  // Register all relevant plugins.
  auto& registry = pxr::PlugRegistry::PlugRegistry::GetInstance();
  std::vector<std::string> json_paths{{
      "openusd_internal/pxr/usd/ar/plugInfo.json",
      "openusd_internal/pxr/usd/ndr/plugInfo.json",
      "openusd_internal/pxr/usd/sdf/plugInfo.json",
      "openusd_internal/pxr/usd/usdGeom/plugInfo.json",
      "openusd_internal/pxr/usd/usd/plugInfo.json",
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
#endif
}
}  // namespace

UsdParser::UsdParser() {
  static const int ignored = []() {
    InitializeOpenUsdLibrary();
    return 0;
  }();
  unused(ignored);
}

UsdParser::~UsdParser() = default;

Eigen::Matrix3d UsdMat3dToEigen(pxr::GfMatrix3d m) {
  Eigen::Matrix3d ret;
  ret << m[0][0], m[0][1], m[0][2],
         m[1][0], m[1][1], m[1][2],
         m[2][0], m[2][1], m[2][2];
  return ret;
}

Eigen::Vector3d UsdVec3dToEigen(pxr::GfVec3d v) {
  return Eigen::Vector3d{ v[0], v[1], v[2] };
}

Eigen::Quaterniond UsdQuatdToEigen(pxr::GfQuatd q) {
  return Eigen::Quaterniond(
    q.GetReal(),
    q.GetImaginary()[0],
    q.GetImaginary()[1],
    q.GetImaginary()[2]);
}

math::RigidTransform<double> GetPrimRigidTransform(
  const pxr::UsdPrim& prim, const UsdStageMetadata& metadata) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d transform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d translation = transform.GetTranslation();
  pxr::GfRotation rotation = transform.GetRotation();
  translation *= metadata.meters_per_unit;

  math::RotationMatrix<double> rotation_matrix(
    UsdQuatdToEigen(rotation.GetQuat()));

  return math::RigidTransform<double>(
    rotation_matrix,
    UsdVec3dToEigen(translation));
}

Eigen::Vector3d GetPrimScale(const pxr::UsdPrim& prim) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d transform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d scale = transform.GetScale();
  return UsdVec3dToEigen(scale);
}

CoulombFriction<double> GetPrimFriction(const pxr::UsdPrim& prim) {
  // TODO(hong-nvidia): Use the prim's friction attributes if has those
  // For now, we just use default friction
  return default_friction();
}

Vector4<double> GetGeomPrimColor(const pxr::UsdPrim& prim) {
  pxr::UsdGeomGprim gprim = pxr::UsdGeomGprim(prim);
  pxr::VtArray<pxr::GfVec3f> colors;
  if (gprim.GetDisplayColorAttr().Get(&colors)) {
    pxr::GfVec3f color = colors[0];
    return Vector4<double>(color[0], color[1], color[2], 1.0);
  } else {
    // Prim does not contain color attribute, use default color instead
    return Vector4<double>(0.5, 0.5, 0.5, 1.0);
  }
}

void ValidatePrimExtent(const pxr::UsdPrim& prim,
  const ParsingWorkspace& workspace, bool check_if_isotropic = false) {
  pxr::VtVec3fArray extent;
  if (!prim.GetAttribute(pxr::TfToken("extent")).Get(&extent)) {
    workspace.diagnostic.Error(fmt::format(
      "Failed to read the extent of prim at {}", prim.GetPath().GetString()));
  }
  const pxr::GfVec3f& lower_bound = extent[0];
  const pxr::GfVec3f& upper_bound = extent[1];
  if (-lower_bound[0] != upper_bound[0] ||
      -lower_bound[1] != upper_bound[1] ||
      -lower_bound[2] != upper_bound[2]) {
    workspace.diagnostic.Error(fmt::format(
      "The extent of the prim at {} is not symmetric",
      prim.GetPath().GetString()));
  }
  if (check_if_isotropic) {
    if (lower_bound[0] != lower_bound[1] ||
        lower_bound[1] != lower_bound[2] ||
        upper_bound[0] != upper_bound[1] ||
        upper_bound[1] != upper_bound[2]) {
      workspace.diagnostic.Error(fmt::format(
        "The extent of the prim at {} should be of the same magnitude across"
        "all three dimensions", prim.GetPath().GetString()));
    }
  }
}

std::unique_ptr<geometry::Shape> CreateGeometryCube(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  ValidatePrimExtent(prim, workspace, true);
  pxr::UsdGeomCube cube = pxr::UsdGeomCube(prim);

  double cube_size = 0;
  if (!cube.GetSizeAttr().Get(&cube_size)) {
    workspace.diagnostic.Error(fmt::format(
      "Failed to read the size of cube at {}", prim.GetPath().GetString()));
  }

  Eigen::Vector3d cube_dimension = GetPrimScale(prim) * cube_size;
  cube_dimension *= metadata.meters_per_unit;
  return std::make_unique<geometry::Box>(cube_dimension);
}

std::unique_ptr<geometry::Shape> CreateGeometrySphere(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  ValidatePrimExtent(prim, workspace, true);
  pxr::UsdGeomSphere sphere = pxr::UsdGeomSphere(prim);

  double sphere_radius = 0;
  if (!sphere.GetRadiusAttr().Get(&sphere_radius)) {
    workspace.diagnostic.Error(fmt::format(
      "Failed to read the radius of sphere at {}",
      prim.GetPath().GetString()));
  }
  sphere_radius *= metadata.meters_per_unit;

  Eigen::Vector3d prim_scale = GetPrimScale(prim);
  if (prim_scale[0] == prim_scale[1] && prim_scale[1] == prim_scale[2]) {
    return std::make_unique<geometry::Sphere>(sphere_radius * prim_scale[0]);
  } else {
    return std::make_unique<geometry::Ellipsoid>(
      sphere_radius * prim_scale[0],
      sphere_radius * prim_scale[1],
      sphere_radius * prim_scale[2]);
  }
}

std::unique_ptr<geometry::Shape> CreateGeometryCapsule(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  return nullptr;
}

std::unique_ptr<geometry::Shape> CreateVisualGeometry(
  const pxr::UsdPrim& prim, const UsdStageMetadata& metadata,
  const ParsingWorkspace& workspace) {
  if (prim.IsA<pxr::UsdGeomCube>()) {
    return CreateGeometryCube(prim, metadata, workspace);
  } else if (prim.IsA<pxr::UsdGeomSphere>()) {
    return CreateGeometrySphere(prim, metadata, workspace);
  } else if (prim.IsA<pxr::UsdGeomCapsule>()) {
    return CreateGeometryCapsule(prim, metadata, workspace);
  } else {
    pxr::TfToken prim_type = prim.GetTypeName();
    workspace.diagnostic.Error(
      fmt::format("Unsupported Prim type: {}", prim_type));
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> CreateCollisionGeometry(
  const pxr::UsdPrim& prim, const UsdStageMetadata& metadata,
  const ParsingWorkspace& workspace) {
  if (prim.IsA<pxr::UsdGeomMesh>()) {
    // See if there is a simplified version of the mesh available
    // for collision detection
    throw std::runtime_error("Mesh parsing is not implemented");
  } else {
    // Assuming all geometries other than mesh are simple enough
    // to be used directly for collision detection
    return CreateVisualGeometry(prim, metadata, workspace);
  }
}

void ProcessStaticCollider(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  MultibodyPlant<double>* plant = workspace.plant;
  math::RigidTransform<double> transform = GetPrimRigidTransform(
    prim, metadata);

  auto collision_geometry = CreateCollisionGeometry(prim, metadata, workspace);
  auto visual_geometry = CreateVisualGeometry(prim, metadata, workspace);

  if (collision_geometry && visual_geometry) {
    plant->RegisterCollisionGeometry(
      plant->world_body(),
      transform,
      *collision_geometry,
      fmt::format("{}-CollisionGeometry", prim.GetPath().GetString()),
      GetPrimFriction(prim));

    plant->RegisterVisualGeometry(
      plant->world_body(),
      transform,
      *visual_geometry,
      fmt::format("{}-VisualGeometry", prim.GetPath().GetString()),
      GetGeomPrimColor(prim));
  }
}

void ProcessPrim(const pxr::UsdPrim& prim, const UsdStageMetadata& metadata,
  const ParsingWorkspace& workspace) {
  drake::log()->info(fmt::format("Processing {}", prim.GetPath().GetString()));
  if (prim.HasAPI(pxr::TfToken("PhysicsCollisionAPI"))) {
    if (prim.HasAPI(pxr::TfToken("PhysicsRigidBodyAPI"))) {
      // ProcessRigidBody(prim, plant);
    } else {
      ProcessStaticCollider(prim, metadata, workspace);
    }
  }
}

UsdStageMetadata GetStageMetadata(pxr::UsdStageRefPtr stage,
  const ParsingWorkspace& workspace) {
  UsdStageMetadata metadata;
  if (!stage->GetMetadata(pxr::TfToken("metersPerUnit"),
    &metadata.meters_per_unit)) {
    workspace.diagnostic.Warning(fmt::format(
      "Failed to read metersPerUnit in stage metadata. "
      "Using the default value ({}) instead.", metadata.meters_per_unit));
  }
  if (!stage->GetMetadata(pxr::TfToken("upAxis"), &metadata.up_axis)) {
    workspace.diagnostic.Warning(fmt::format(
      "Failed to read upAxis in stage metadata. "
      "Using the default value ({}) instead.", metadata.up_axis));
  }
  if (metadata.up_axis != "Z") {
    throw std::runtime_error("Only Z-up stages are supported currently");
  }
  return metadata;
}

std::optional<ModelInstanceIndex> UsdParser::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  unused(data_source, model_name, parent_model_name, workspace);
  throw std::runtime_error("UsdParser::AddModel is not implemented");
}

std::vector<ModelInstanceIndex> UsdParser::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  unused(parent_model_name);

  if (data_source.IsContents()) {
    throw std::runtime_error(
      "Ingesting raw USD content from DataSource is not implemented");
  }

  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(
    data_source.GetAbsolutePath());
  if (!stage) {
    workspace.diagnostic.Error(fmt::format("Failed to open USD stage",
      data_source.filename()));
  }

  UsdStageMetadata metadata = GetStageMetadata(stage, workspace);

  for (pxr::UsdPrim prim : stage->Traverse()) {
    ProcessPrim(prim, metadata, workspace);
  }

  // Returning an empty vector, since right now we only support static
  // colliders attached to the world body (i.e., no actual models are
  // involved).
  return std::vector<ModelInstanceIndex>();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

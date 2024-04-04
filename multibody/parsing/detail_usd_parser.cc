#include "drake/multibody/parsing/detail_usd_parser.h"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/gprim.h"
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

Eigen::Matrix3d UsdMat3dToEigenMat3d(pxr::GfMatrix3d m) {
  Eigen::Matrix3d ret;
  // TODO(hong-nvidia): check if we need to transpose the matrix
  ret << m[0][0], m[0][1], m[0][2],
         m[1][0], m[1][1], m[1][2],
         m[2][0], m[2][1], m[2][2];
  return ret;
}

Eigen::Vector3d UsdVec3dtoEigenVec3d(pxr::GfVec3d v) {
  return Eigen::Vector3d{ v[0], v[1], v[2] };
}

math::RigidTransform<double> GetPrimRigidTransform(
  const pxr::UsdPrim& prim, const UsdStageMetadata& metadata) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d xform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());
  pxr::GfVec3d translation = xform_matrix.ExtractTranslation();
  translation *= metadata.meters_per_unit;
  pxr::GfMatrix3d rotation = xform_matrix.ExtractRotationMatrix()
    .GetOrthonormalized();  // The extracted matrix contains scaling so
    // we have to orthonormalize it

  return math::RigidTransform<double>(
    math::RotationMatrixd(UsdMat3dToEigenMat3d(rotation)),
    UsdVec3dtoEigenVec3d(translation));
}

Eigen::Vector3d CalculateCubeDimension(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  pxr::UsdGeomCube cube = pxr::UsdGeomCube(prim);

  // TODO(hong-nvidia): make sure that the extent of the cube is symmetric

  double cube_size = 0;
  if (!cube.GetSizeAttr().Get(&cube_size)) {
    workspace.diagnostic.Error(fmt::format(
      "Failed to read the size of cube at {}", prim.GetPath().GetString()));
  }

  pxr::GfVec3f prim_scale;
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);
  if (!xformable.GetScaleOp().Get(&prim_scale)) {
    workspace.diagnostic.Error(fmt::format(
      "Failed to read the scale of prim at {}", prim.GetPath().GetString()));
  }

  Eigen::Vector3d dimension = Eigen::Vector3d(prim_scale[0] * cube_size,
                                              prim_scale[1] * cube_size,
                                              prim_scale[2] * cube_size);
  dimension *= metadata.meters_per_unit;
  return dimension;
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

void ProcessStaticColliderCube(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  MultibodyPlant<double>* plant = workspace.plant;
  Eigen::Vector3d cube_dimension = CalculateCubeDimension(prim, metadata,
    workspace);
  math::RigidTransform<double> transform = GetPrimRigidTransform(
    prim, metadata);

  plant->RegisterCollisionGeometry(
    plant->world_body(),
    transform,
    geometry::Box(cube_dimension),
    fmt::format("{}-CollisionGeometry", prim.GetPath().GetString()),
    GetPrimFriction(prim));

  plant->RegisterVisualGeometry(
    plant->world_body(),
    transform,
    geometry::Box(cube_dimension),
    fmt::format("{}-VisualGeometry", prim.GetPath().GetString()),
    GetGeomPrimColor(prim));
}

void ProcessStaticCollider(const pxr::UsdPrim& prim,
  const UsdStageMetadata& metadata, const ParsingWorkspace& workspace) {
  if (prim.IsA<pxr::UsdGeomCube>()) {
    ProcessStaticColliderCube(prim, metadata, workspace);
  } else if (prim.IsA<pxr::UsdGeomSphere>()) {
    // ProcessStaticColliderSphere(prim, workspace);
    throw std::runtime_error("UsdGeomSphere parsing is not yet implemented");
  } else {
    pxr::TfToken prim_type = prim.GetTypeName();
    workspace.diagnostic.Error(
      fmt::format("Unsupported Prim type: {}", prim_type));
  }
}

void ProcessPrim(const pxr::UsdPrim& prim, const UsdStageMetadata& metadata,
  const ParsingWorkspace& workspace) {
  drake::log()->info("Processing " + prim.GetPath().GetString());

  if (prim.HasAPI(pxr::TfToken("PhysicsCollisionAPI"))) {
    if (prim.HasAPI(pxr::TfToken("PhysicsRigidBodyAPI"))) {
      // ProcessRigidBody(prim, plant);
    } else {
      ProcessStaticCollider(prim, metadata, workspace);
    }
  }
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

  UsdStageMetadata metadata;
  if (!stage->GetMetadata(pxr::TfToken("metersPerUnit"),
      &metadata.meters_per_unit)) {
    workspace.diagnostic.Warning(fmt::format(
      "Failed to read metersPerUnit in stage metadata. "
      "Using the default value ({}) instead.", metadata.meters_per_unit));
  }

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

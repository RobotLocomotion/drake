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

void ProcessStaticCollider(
  const pxr::UsdPrim& prim, const ParsingWorkspace& workspace) {
  MultibodyPlant<double>* plant = workspace.plant;
  auto diag = workspace.diagnostic;

  DRAKE_ASSERT(prim.IsA<pxr::UsdGeomXformable>());
  if (prim.IsA<pxr::UsdGeomCube>()) {
    pxr::UsdGeomCube cube = pxr::UsdGeomCube(prim);

    pxr::GfVec3f scale;
    pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);
    if (!xformable.GetScaleOp().Get(&scale)) {
      workspace.diagnostic.Error(fmt::format(
        "Failed to read the scale of prim at {}", prim.GetPath().GetString()));
    }
    // Reset the scale to 1 so that ComputeLocalToWorldTransform below does not
    // include scaling
    xformable.GetScaleOp().Set(pxr::GfVec3f(1.f));

    pxr::GfMatrix4d xform_matrix = xformable.ComputeLocalToWorldTransform(
      pxr::UsdTimeCode::Default());
    pxr::GfMatrix3d rotation = xform_matrix.ExtractRotationMatrix();
    pxr::GfVec3d translation = xform_matrix.ExtractTranslation();

    math::RigidTransform<double> transform(
      math::RotationMatrixd(UsdMat3dToEigenMat3d(rotation)),
      UsdVec3dtoEigenVec3d(translation));

    CoulombFriction<double> friction = default_friction();
    plant->RegisterCollisionGeometry(
      plant->world_body(),
      transform,
      geometry::Box(scale[0], scale[1], scale[2]),
      fmt::format("{}-CollisionGeometry", prim.GetPath().GetString()),
      friction);

    const Vector4<double> color_white(0.9, 0.9, 0.9, 1.0);
    plant->RegisterVisualGeometry(
      plant->world_body(),
      transform,
      geometry::Box(scale[0], scale[1], scale[2]),
      fmt::format("{}-VisualGeometry", prim.GetPath().GetString()),
      color_white);
  } else {
    pxr::TfToken prim_type = prim.GetTypeName();
    diag.Error(fmt::format("Unsupported Prim type: {}", prim_type));
  }
}

void ProcessPrim(const pxr::UsdPrim& prim, const ParsingWorkspace& workspace) {
  drake::log()->info("Processing " + prim.GetPath().GetString());

  if (prim.HasAPI(pxr::TfToken("PhysicsCollisionAPI"))) {
    if (prim.HasAPI(pxr::TfToken("PhysicsRigidBodyAPI"))) {
      // ProcessRigidBody(prim, plant);
    } else {
      ProcessStaticCollider(prim, workspace);
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

  for (pxr::UsdPrim prim : stage->Traverse()) {
    ProcessPrim(prim, workspace);
  }

  // TODO(hong-nvidia) Returning an empty vector for now as a placeholder
  return std::vector<ModelInstanceIndex>();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

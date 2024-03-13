#include "drake/multibody/parsing/detail_usd_parser.h"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
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

void ProcessPrim(const pxr::UsdPrim& prim, MultibodyPlant<double>* plant) {
  drake::log()->info("Processing " + prim.GetPath().GetString());

  if (prim.HasAPI(pxr::TfToken("PhysicsCollisionAPI"))) {
    pxr::TfToken prim_type = prim.GetTypeName();
    if (prim_type == "Cube") {
      drake::log()->info("Cube");

      math::RigidTransform<double> transform;
      CoulombFriction<double> friction = default_friction();
      const Vector4<double> color_white(0.9, 0.9, 0.9, 1.0);
      double scale_x = 1;
      double scale_y = 1;
      double scale_z = 1;

      plant->RegisterCollisionGeometry(
        plant->world_body(),
        transform,
        geometry::Box(scale_x, scale_y, scale_z),
        fmt::format("{}-CollisionGeometry", prim.GetPath().GetString()),
        friction);
      plant->RegisterVisualGeometry(
        plant->world_body(),
        transform,
        geometry::Box(scale_x, scale_y, scale_z),
        fmt::format("{}-VisualGeometry", prim.GetPath().GetString()),
        color_white);
    } else {
      throw std::runtime_error(
        fmt::format("Unsupported Prim type for PhysicsCollisionAPI: {}",
        prim_type));
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
    throw std::runtime_error("Failed to open USD stage");
  }

  for (pxr::UsdPrim prim : stage->Traverse()) {
    ProcessPrim(prim, workspace.plant);
  }

  // TODO(hong-nvidia) Returning an empty vector for now as a placeholder
  return std::vector<ModelInstanceIndex>();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

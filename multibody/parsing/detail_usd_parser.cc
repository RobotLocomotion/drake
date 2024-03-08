#include "drake/multibody/parsing/detail_usd_parser.h"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/stage.h"

#include "drake/common/find_runfiles.h"
#include "drake/common/unused.h"

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
  if (data_source.IsContents()) {
    throw std::runtime_error("UsdParser::AddAllModels - Ingesting raw USD"
      "content from DataSource is not implemented");
  }

  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(data_source.GetAbsolutePath());
  if (!stage) {
    drake::log()->error("Failed to open stage");
  } else {
    drake::log()->info("Sucessfully opened stage");
  }

  unused(parent_model_name, workspace);
  
  // TODO(hong-nvidia) Returning an empty vector for now as a placeholder
  return std::vector<ModelInstanceIndex>();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

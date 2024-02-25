/* clang-format off to disable clang-format-includes */
#include "drake/multibody/parsing/detail_usd_parser.h"
/* clang-format on */

#include <stdexcept>

#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace internal {

UsdParser::UsdParser() = default;

UsdParser::~UsdParser() = default;

std::optional<ModelInstanceIndex> UsdParser::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  unused(data_source, model_name, parent_model_name, workspace);
  throw std::runtime_error(
      "UsdParser is not available because WITH_USD is not ON");
}

std::vector<ModelInstanceIndex> UsdParser::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  unused(data_source, parent_model_name, workspace);
  throw std::runtime_error(
      "UsdParser is not available because WITH_USD is not ON");
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

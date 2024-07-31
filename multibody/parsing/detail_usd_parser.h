#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

class UsdParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UsdParserWrapper);

  UsdParserWrapper();

  ~UsdParserWrapper() final;

  std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;

  // Initializes the OpenUSD library by registering relevant plugins.
  // The function should be invoked once and only once before importing any
  // USD asset.
  static void InitializeOpenUsdLibrary();
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

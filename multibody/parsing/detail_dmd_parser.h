#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/model_directives.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(rpoyner-tri): diagnostic policy?
parsing::ModelDirectives LoadModelDirectives(const DataSource& data_source);

std::vector<parsing::ModelInstanceInfo> ProcessModelDirectives(
    const parsing::ModelDirectives& directives,
    const std::optional<std::string>& scope_name,
    const ParsingWorkspace& workspace);

// Parses DMD (Drake Model Directive) data.
class DmdParser : public ParserInterface {
 public:
  DmdParser();
  ~DmdParser() override;
  std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& scope_name,
      const ParsingWorkspace& workspace) override;

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& scope_name,
      const ParsingWorkspace& workspace) override;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

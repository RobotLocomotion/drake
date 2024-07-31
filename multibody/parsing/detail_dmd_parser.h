#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/model_directives.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace internal {

// DMD adds one small bit of sugar to Drake's ScopedName idiom:  The name
// "world" always refers to the world regardless of any enclosing scopes.
ScopedName DmdScopedNameJoin(const std::string& namespace_name,
                             const std::string& element_name);

// TODO(#18052): diagnostic policy?
parsing::ModelDirectives LoadModelDirectives(const DataSource& data_source);

std::vector<parsing::ModelInstanceInfo> ParseModelDirectives(
    const parsing::ModelDirectives& directives,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

// Parses DMD (Drake Model Directive) data.
class DmdParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DmdParserWrapper);
  DmdParserWrapper();
  ~DmdParserWrapper() final;
  std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

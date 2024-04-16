#include "drake/multibody/parsing/detail_composite_parse.h"

#include "drake/multibody/parsing/detail_parser_internal_data.h"
#include "drake/multibody/parsing/detail_select_parser.h"

namespace drake {
namespace multibody {
namespace internal {

std::unique_ptr<CompositeParse> CompositeParse::MakeCompositeParse(
    Parser* parser) {
  DRAKE_DEMAND(parser != nullptr);
  // This slightly odd spelling allows access to the private constructor.
  return std::unique_ptr<CompositeParse>(new CompositeParse(parser));
}

CompositeParse::CompositeParse(Parser* parser)
    : parser_(parser),
      resolver_(&parser->plant()),
      options_({parser->GetAutoRenaming()}),
      workspace_(options_, parser->package_map(), parser->diagnostic_policy_,
                 &parser->plant(), &resolver_, SelectParser) {}

CompositeParse::~CompositeParse() {
  resolver_.Resolve(workspace_.diagnostic);

  // Convert scoped names into InstancedNames for storage in between parses.
  auto convert = [this](const std::string& input)
                 -> InstancedName {
    auto plant = workspace_.plant;
    InstancedName result;
    auto scoped = ScopedName::Parse(input);
    if (plant->HasModelInstanceNamed(scoped.get_namespace())) {
      result.index = plant->GetModelInstanceByName(scoped.get_namespace());
    }
    result.name = scoped.get_element();
    return result;
  };

  // Merge the groups found in this parse into the accumulated groups over the
  // lifetime of the parser.
  MergeCollisionFilterGroups<internal::InstancedName, std::string>(
      &parser_->data_->collision_filter_groups_storage_,
      resolver_.GetCollisionFilterGroups(),
      convert);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

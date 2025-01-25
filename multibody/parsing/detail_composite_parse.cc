#include "drake/multibody/parsing/detail_composite_parse.h"

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
                 parser->builder(), &parser->plant(), &resolver_,
                 SelectParser) {}

CompositeParse::~CompositeParse() = default;

void CompositeParse::Finish() {
  parser_->ResolveCollisionFilterGroupsFromCompositeParse(&resolver_);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

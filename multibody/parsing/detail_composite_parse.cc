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
    : resolver_(&parser->plant()),
      workspace_(parser->package_map(), parser->diagnostic_policy_,
                 &parser->plant(), &resolver_, SelectParser) {}

CompositeParse::~CompositeParse() {
  resolver_.Resolve(workspace_.diagnostic);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

#include "drake/multibody/parsing/detail_composite_parse.h"

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
    : parser_(parser), resolver_(&parser->plant()) {}

CompositeParse::~CompositeParse() {
  resolver_.Resolve(parser_->diagnostic_policy_);
}

CollisionFilterGroupResolver& CompositeParse::collision_resolver() {
  return resolver_;
}

ModelInstanceIndex CompositeParse::AddModelFromFile(
    const std::string& file_name, const std::string& model_name) {
  return parser_->CompositeAddModelFromFile(file_name, model_name, this);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <tinyxml2.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_common.h"

namespace drake {
namespace multibody {
namespace internal {

// An object that contains bookkeeping that should span the parsing of
// multiple models. Since it internally references the parser passed to it at
// construction, its lifetime must be shorter than that of the parser.
class CompositeParse {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompositeParse)

  explicit CompositeParse(std::reference_wrapper<Parser> parser)
      : parser_(parser.get()), resolver_(&parser_.plant()) {}
  ~CompositeParse() { resolver_.Resolve(parser_.diagnostic_policy_); }

  CollisionFilterGroupResolver& collision_resolver() { return resolver_; }
  // TODO(rpoyner-tri): add some way to get more expressive diagnostics.

  ModelInstanceIndex AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
    return parser_.CompositeAddModelFromFile(file_name, model_name, this);
  }
  // TODO(rpoyner-tri): add model parsing methods as needed.

 private:
  Parser& parser_;
  CollisionFilterGroupResolver resolver_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

#pragma once

#include <memory>
#include <string>

#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace internal {

// An object that contains bookkeeping that should span the parsing of
// multiple models. Since it internally references the parser passed to it at
// construction, its lifetime must be shorter than that of the parser.
class CompositeParse {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompositeParse)

  // Build and return a new composite parse object.
  static std::unique_ptr<CompositeParse> MakeCompositeParse(Parser* parser);

  ~CompositeParse();

  CollisionFilterGroupResolver& collision_resolver();
  // TODO(rpoyner-tri): add some way to get more expressive diagnostics.

  ModelInstanceIndex AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name);
  // TODO(rpoyner-tri): add model parsing methods as needed.

 private:
  explicit CompositeParse(Parser* parser);

  Parser* const parser_;
  CollisionFilterGroupResolver resolver_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

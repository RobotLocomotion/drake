#pragma once

#include <memory>
#include <string>

#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace internal {

// An object that contains bookkeeping that should span the parsing of
// multiple models. Since it internally references the parser passed to it at
// construction, its lifetime must be shorter than that of the parser.
class CompositeParse {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompositeParse);

  // Build and return a new composite parse object.
  static std::unique_ptr<CompositeParse> MakeCompositeParse(Parser* parser);

  // Prior to destroying this object, users of this class should call Finish()
  // exactly once, to complete our interaction with the Parser. This operation
  // can throw an exception, so must NOT be done during stack unwinding.
  void Finish();

  ~CompositeParse();

  // @returns a const reference to a workspace. Note that some objects pointed
  // to by the workspace may still be mutated; see ParsingWorkspace for
  // details.
  const ParsingWorkspace& workspace() const { return workspace_; }

 private:
  explicit CompositeParse(Parser* parser);

  Parser* const parser_;
  CollisionFilterGroupResolver resolver_;
  const ParsingOptions options_;
  const ParsingWorkspace workspace_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

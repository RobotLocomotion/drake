#pragma once

#include <string>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"

namespace drake {
namespace multibody {
namespace internal {

// This function matches the drake::multibody::internal::ParserSelector functor
// type. If a matching parser can't be found, this implementation emits an
// error and returns a do-nothing object, whose methods do nothing at all.
ParserInterface& SelectParser(const drake::internal::DiagnosticPolicy& policy,
                              const std::string& filename);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

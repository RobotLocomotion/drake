#pragma once

#include <string>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"

namespace drake {
namespace multibody {
namespace internal {

ParserInterface* SelectParser(const drake::internal::DiagnosticPolicy& policy,
                              const std::string& file_name);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
